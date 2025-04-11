/*!
    \file
    \brief  Implements a fully asynchronous SMS send/receive in PDU mode for SIM7000 modem class
    \author Flying Domotic
    \date   March 31st, 2025
*/

#include <FF_Sim7000.h>
#include <FF_Trace.h>
#include <time.h>
#include <mktime.h>
#ifdef ESP8266
    #include <NtpClientLib.h>
#endif

#include <pdulib.h>                                                 // https://github.com/mgaman/PDUlib

struct initStepsStruct {
    void (FF_Sim7000::*nextStep)(void);
    char command[20];
    char waitFor[10];
    unsigned long timeout;
    uint8_t repeat;
};

uint8_t stepPtr;                                                    // Pointer into initSteps table
uint8_t stepRepeatCount;                                            // Count of repeat already done
uint8_t stepMaxRepeatcount;                                         // Max repeat count for this command

// Table containing init modem data (next step to run, data to send, data to wait for, timeout, repeat count)
#define STEP_SIZE 14
struct initStepsStruct initSteps[STEP_SIZE] = {
    {nullptr,               "AT",                   "",             1000,                9}, // Begin, send AT (up to 10 times, 1s interval)
    {nullptr,               "AT+IPR=115200",        "",             SIM7000_CMD_TIMEOUT, 0}, // Modem comm speed is 115200
    {nullptr,               "ATE0",                 "",             SIM7000_CMD_TIMEOUT, 0}, // Echo off
    {nullptr,               "AT+CMEE=2",            "",             SIM7000_CMD_TIMEOUT, 0}, // Return detailled error messages
    {nullptr,               "AT+CMGF=0",            "",             SIM7000_CMD_TIMEOUT, 0}, // Set SMS mode = PDU (0)
    {nullptr,               "AT+CNMP=51" ,           "",             SIM7000_CMD_TIMEOUT, 0}, // Prefered network mode = auto (2)
    {nullptr,               "AT+CREG=2",            "",             SIM7000_CMD_TIMEOUT, 0}, // Verbose register network
    {nullptr,               "AT+CSDH=1",            "",             SIM7000_CMD_TIMEOUT, 0}, // Show SMS headers
    {nullptr,               "AT+CMGD=1,4",          "",             10000,               0}, // Delete all pending messages
    {nullptr,               "AT+CNMI=2,2,0,2,0",    "",             SIM7000_CMD_TIMEOUT, 0}, // New messages indication
    {nullptr,               "AT+CREG?",             "",             SIM7000_CMD_TIMEOUT, 0}, // Ask for network register status
    {nullptr,               "AT+CLTS=1",            "",             SIM7000_CMD_TIMEOUT, 0}, // Ask for local time
    {nullptr,               "AT+CSCA?",             CSCA_INDICATOR, 10000,               0}, // Ask for CSA number
    {&FF_Sim7000::gotSca,   "",                     "",             SIM7000_CMD_TIMEOUT, 0}, // We got SCA number, save it for PDU
};

#define PDU_BUFFER_LENGTH 1024                                      // Max workspace length
PDU smsPdu = PDU(PDU_BUFFER_LENGTH);                                // Instantiate PDU class

#ifdef FF_SIM7000_USE_SOFTSERIAL                                    // Define FF_SIM7000_USE_SOFTSERIAL to use SofwareSerial instead of Serial
    #include <SoftwareSerial.h>
    SoftwareSerial Sim7000Serial;                                   // We use software serial to keep Serial usable
    #warning Using SoftwareSerial may be unreliable at high speed!
#else
    #if defined(FF_SIM7000_USE_SERIAL1)
        #define Sim7000Serial Serial1                               // Use Serial1
    #elif defined(FF_SIM7000_USE_SERIAL2)
        #define Sim7000Serial Serial2                               // Use Serial2
    #else
        #define Sim7000Serial Serial                                // Use Serial for Sim7000lib
    #endif
#endif

// Class constructor : init some variables
FF_Sim7000::FF_Sim7000() {
    restartNeeded = false;
    gsmStatus = SIM7000_NEED_INIT;
    restartReason = gsmStatus;
    smsReady = false;
    inReceive = false;
    gsmIdle = SIM7000_STARTING;
    debugFlag = false;
    traceFlag = false;
    traceEnterFlag = false;
    firstInitDone = false;
    modemSpeaking = false;
    nextLineIsSmsMessage = false;
    commandCount = 0;
    resetCount = 0;
    smsReadCount = 0;
    smsForwardedCount = 0;
    smsSentCount = 0;
    lastReceivedNumber = "";
    lastReceivedDate = "";
    lastReceivedMessage = "";
    lastSentNumber = "";
    lastSentDate = "";
    lastSentMessage = "";
    ignoreErrors = false;
    startTime = 0;
    restartCount = 0;
    nextStepCb = NULL;
    readSmsCb = NULL;
    sendSmsCb = NULL;
    recvLineCb = NULL;
    index = 0;
    gsmTimeout = 0;
    inWait = false;
    inWaitSmsReady = false;
    memset(lastAnswer, 0, sizeof(lastAnswer));
    memset(expectedAnswer, 0, sizeof(expectedAnswer));
    memset(lastCommand, 0, sizeof(lastCommand));
    smsMsgId = 0;
    powerStepDuration[0] = 1500;                                    // High for 1,5s (will reset)
    powerStepDuration[1] = 2000;                                    // Low for 2s (will give time for modem to switch off)
    powerStepDuration[2] = 1500;                                    // High for 1,5s (will reset)
    powerStepDuration[3] = 10000;                                   // Low for 10s (will give time for modem to switch on)
    powerStepDuration[4] = 0;                                       // End of table
}

/*!

    \brief  Define power key level

    Define power key signal level (high or low)

    \return none

*/
void FF_Sim7000::setPowerPin(void) {
    uint8_t powerState = (powerStep & 1)? SIM7000_PIN_INACTIVE : SIM7000_PIN_ACTIVE;
    trace_debug_P("Modem power step %d, level %d for %d ms", powerStep, powerState, powerStepDuration[powerStep]);
    digitalWrite(modemPowerPin, powerState);
    powerStepStartTime = millis();
}

/*!

    \brief  Initialize the GSM connection

    Initialize GSM connection

    \param[in]  baudRate: modem speed (in bauds). Modem will be properly switched to this speed if needed
    \param[in]  rxPin: ESP pin used to receive data from modem (connected to Sim7000 TX)
    \param[in]  txPin: ESP pin used to send data to modem (connected to Sim7000 RX)
    \param[in]  powerPin: ESP pin used to power up/down modem (connected to Sim7000 power key)
    \return none

*/
void FF_Sim7000::begin(long baudRate, int8_t rxPin, int8_t txPin, int8_t powerPin) {
    if (traceFlag) enterRoutine(__func__);
    trace_debug_P("Sim7000 begin", NULL);
    restartNeeded = false;
    inReceive = false;
    inWait = false;
    gsmIdle = SIM7000_STARTING;
    // Save RX pin, TX pin and requested speed
    modemRxPin = rxPin;
    modemTxPin = txPin;
    modemPowerPin = powerPin;
    modemSpeed = baudRate;
    modemConnected = false;
    powerStep = 0;                                                  // Reset power step
    // If no power pin defined, open modem directly
    if (modemPowerPin < 0) {
        powerStepDuration[0] = 0;                                   // End of table
        powerStepStartTime = 0;                                     // Reset start time
        open();
    } else {
        // Modem pin defined
        pinMode(modemPowerPin, OUTPUT);
        //  If modem was already initialized and didn't answer previous session
        if (firstInitDone && ! modemSpeaking) {
            // No answer can be due to a power down issue, just toggle power once
            powerStep = 2;
        }
        setPowerPin();
    }
    // Flag init done
    firstInitDone = true;
    // Set modem don't spoke this session
    modemSpeaking = false;
}

/*!

    \brief  Open GSM serial

    Open the modem connection at given baud rate

    \return none

*/

void FF_Sim7000::open() {
    if (traceFlag) enterRoutine(__func__);
    trace_debug_P("Opening modem", NULL);
    // Open modem at requested speed initially
    openModem(modemSpeed);
    // Init pointer and start modem init process
    stepPtr = 0;
    stepMaxRepeatcount = 0;
    stepRepeatCount = 0;
    sendCurrentInitStep();
}

/*!

    \brief  Modem loop (should be called in main loop)

    This routine should be called at regular interval in order for the modem code to run (as we're asynchronous)

    \param  none
    \return none

*/
void FF_Sim7000::doLoop(void) {
    if (traceFlag) enterRoutine(__func__);
    // Are we in modem power steps?
    if (powerStepStartTime) {
        // Is current step finished
        if ((millis() - powerStepStartTime) >= powerStepDuration[powerStep]) {
            // Set power pin level
            powerStep++;
            setPowerPin();
            // Is this the last power step?
            if (!powerStepDuration[powerStep]) {
                // Last step, clear step begin time
                powerStepStartTime = 0;
                // Release power key
                pinMode(modemPowerPin, OUTPUT_OPEN_DRAIN);
                // Open Serial modem port
                open();
            }
        }
    } else {
        // Read modem until \n (LF) character found, removing \r (CR)
            size_t answerLen = strlen(lastAnswer);                  // Get answer length
            while (Sim7000Serial.available()) {
                // Flag modem speaking
                modemSpeaking = true;
                char c = Sim7000Serial.read();
                // Skip NULL and CR characters
                if (c != 0 && c != 13) {
                    if (answerLen >= sizeof(lastAnswer)-2) {
                        trace_error_P("Answer too long: >%s<", lastAnswer);
                        // Answer is too long
                        gsmStatus = SIM7000_TOO_LONG;
                        resetLastAnswer();
                        return;
                    }
                    // Do we have an answer?
                    if (c == 10) {
                        #ifdef FF_SIM7000_DUMP_MESSAGE_ON_SERIAL
                            Serial.print("<LF>");
                        #endif
                        // Do we have an "CREG" message or answer?
                        char *ptrStr = strstr(lastAnswer, CREG_MSG);
                        if (ptrStr) {
                            // Do we just sent a CREG request?
                            if (strstr(lastCommand, CREG_QUERY)) {
                                // We asked for CREG, there's a single char plus a comma to skip
                                ptrStr += sizeof(CREG_MSG)+1;
                            } else {
                                // This is an unsolicited CREG, skip only message length (except ending \0)
                                ptrStr += sizeof(CREG_MSG)-1;
                            }
                            // Extract number after CREG_MSG
                            char result = ptrStr[0];
                            if (debugFlag) trace_debug_P("Got %s, state: %c", lastAnswer, result);
                            smsReady = (result == '1' || result == '5');
                            resetLastAnswer();
                            return;
                        }
                        // Do we have a time message?
                        #ifdef FF_SIM6000_SET_TIME_FROM_GSM_NETWORK
                        ptrStr = strstr(lastAnswer, GSM_TIME);
                        if (ptrStr) {
                            // Format in doc: *PSUTTZ: <year>,<month>,<day>,<hour>,<min>,<sec>,"<timezone>",<dst>
                            // Message received: *PSUTTZ: 25/04/02,09:49:27","+08",1
                            // As you can see, formats are not really the same...
                            // We'll try to adapt to some kind of "artistic" format, loading only useful characters
                            // Extract date part
                            char currentDate[35];
                            memset(currentDate, 0, sizeof(currentDate));
                            uint8_t outPtr = 0;
                            // Skip GSM_TIME marker
                            ptrStr += sizeof(GSM_TIME) -1 ;
                            char c = ptrStr[0];
                            // Run until end of string
                            bool scanOk = true;
                            while (c && scanOk) {
                                //  Replace / and : by ,
                                if (c == '/' || c == ':') {
                                    currentDate[outPtr++] = ',';
                                //  Store 0 to 9, + , - and ,
                                } else if ((c >= '0' && c <= '9') || c == '+' || c == '-' || c == ',') {
                                    currentDate[outPtr++] = c;
                                // Skip "
                                } else if (c != '"') {
                                    // Reject illegal characters
                                    if (debugFlag) trace_debug_P("Illegal character 0x%x found in %s", c, lastAnswer);
                                    scanOk = false;
                                }
                                // Next char in answer
                                ptrStr++;
                                c = ptrStr[0];
                                // Exit loop if currentDate overflow
                                if (outPtr >= sizeof(currentDate)) {
                                    if (debugFlag) trace_debug_P("Answer %s is too long (%d-%s)", lastAnswer, outPtr, currentDate);
                                    scanOk = false;
                                }
                            }
                            // If returned data is ok
                            if (scanOk) {
                                // Now, message should looks like 25,04,02,09,49,27,+08,1
                                // We should extract 8 values (year, month, day, hour, minute, second, quarters to GMT, DST flag)
                                #define TOKEN_COUNT 8
                                int values[TOKEN_COUNT];
                                #define TM_YEAR 0
                                #define TM_MONTH 1
                                #define TM_DAY 2
                                #define TM_HOUR 3
                                #define TM_MIN 4
                                #define TM_SEC 5
                                #define TM_QUARTERS_TO_UTC 6
                                #define TM_IS_DST 7
                                // Extract first token
                                char *token = strtok (currentDate, ",");
                                // Scan the 8 tokens
                                for (int i = 0; i < TOKEN_COUNT; i++) {
                                    // If extraction is still ok
                                    if (scanOk) {
                                        // Do we have a token?
                                        if (token != NULL) {
                                            values[i] = atoi(token);
                                            token = strtok(NULL, ",");
                                        } else {
                                            // No more token, give error
                                            if (debugFlag) trace_debug_P("Token %d not found", i);
                                            scanOk = 0;
                                        }
                                    }
                                }
                                // Still ok?
                                if (scanOk) {
                                    // Set time
									uint32_t unixTime = (long) unixTimeInSeconds(values[TM_SEC], values[TM_MIN], values[TM_HOUR], values[TM_DAY], values[TM_MONTH], values[TM_YEAR] + 2000);
									timeval epoch = {(long) unixTime, 0};
									if (debugFlag) {
										tm timeinfo;
										getLocalTime(&timeinfo, 250);
										char dateBefore[25];
										strftime(dateBefore, sizeof(dateBefore), "%Y/%m/%d %H:%M:%S", &timeinfo);
										settimeofday((const timeval*)&epoch, 0);
										getLocalTime(&timeinfo, 250);
										char dateAfter[25];
										strftime(dateAfter, sizeof(dateAfter), "%Y/%m/%d %H:%M:%S", &timeinfo);
										trace_debug_P("Date changed from %s to %s", dateBefore, dateAfter);

									} else {
										settimeofday((const timeval*)&epoch, 0);
									}
                                    resetLastAnswer();
                                    return;
                                }
                            }
                            resetLastAnswer();
                        }
                        #endif
                        if (inReceive) {                            // Are we waiting for a command answer?
                            // Is this the expected answer?
                            bool isDefaultAnwer = !(strcmp(expectedAnswer, DEFAULT_ANSWER));
                            if ((isDefaultAnwer && !strcmp(lastAnswer, expectedAnswer)) || (!isDefaultAnwer && strstr(lastAnswer, expectedAnswer))) {
                                if (debugFlag) trace_debug_P("Reply in %d ms: >%s<", millis() - startTime, lastAnswer);
                                gsmStatus = SIM7000_OK;
                                if (nextStepCb) {                   // Do we have another callback to execute?
                                    (this->*nextStepCb)();          // Yes, do it
                                return;
                            }
                                    setIdle();                      // No, we just finished.
                                return;
                            }
                            if (!ignoreErrors) {                    // Should we ignore errors?
                                // No, check for CMS/CME error
                                if (strstr(lastAnswer,"+CMS ERROR") || strstr(lastAnswer,"+CME ERROR")) {
                                    // This is a CMS or CME answer
                                    trace_error_P("Error answer: >%s< after %d ms, command was %s", lastAnswer, millis() - startTime, lastCommand);
                                    gsmStatus = SIM7000_CM_ERROR;
                                    restartNeeded = true;
                                    restartReason = gsmStatus;
                                    setIdle();
                                    return;
                                }
                            }
                        }
                        if (strlen(lastAnswer)) {                   // Answer is not null
                            if (nextLineIsSmsMessage) {             // Are we receiving a SMS message?
                                if (debugFlag) trace_debug_P("Message is >%s<", lastAnswer);    // Display cleaned message
                                readSmsMessage(lastAnswer);         // Yes, read it
                                resetLastAnswer();
                                nextLineIsSmsMessage = false;       // Clear flag
                                return;
                            } else {                                // Not in SMS reception
                                if (strstr(lastAnswer, SMS_INDICATOR)) {// Is this indicating an SMS reception?
                                    if (debugFlag) trace_debug_P("Indicator is >%s<", lastAnswer);  // Display cleaned message
                                    // Load last command with indicator
                                    strncpy(lastCommand, lastAnswer, sizeof(lastCommand));
                                    readSmsHeader(lastAnswer);
                                    resetLastAnswer();
                                    inReceive = true;
                                    gsmTimeout = 2000;
                                    startTime = millis();
                                    return;
                                } else {                            // Can't understand received data
                                    if (debugFlag) trace_debug_P("Ignoring >%s<", lastAnswer);  // Display cleaned message
                                    if (recvLineCb) (*recvLineCb)(lastAnswer);  // Activate callback with answer
                                    resetLastAnswer();
                                    return;
                                }
                            }
                        }
                    } else {
                        // Add received character to lastAnswer
                        #ifdef FF_SIM7000_DUMP_MESSAGE_ON_SERIAL
                            Serial.print(c);
                        #endif
                        lastAnswer[answerLen++] = c;                // Copy character
                        lastAnswer[answerLen] = 0;                  // Just in case we forgot cleaning buffer
                        // Check for one character answer (like '>' when sending SMS) which have no <CR><LF>
                        if (strlen(expectedAnswer) == 1 && c == expectedAnswer[0]) {
                            if (debugFlag) trace_debug_P("Reply in %d ms: >%s<", millis() - startTime, lastAnswer);
                            gsmStatus = SIM7000_OK;
                            if (nextStepCb) {                       // Do we have another callback to execute?
                                (this->*nextStepCb)();              // Yes, do it
                            return;
                        }
                                setIdle();                          // No, we just finished.
                            return;
                        }
                    }
                #ifdef FF_SIM7000_DUMP_MESSAGE_ON_SERIAL
                } else {
                    if (c == 0) {
                        Serial.print("<NULL>");
                    } else if (c == 13 ) {
                        Serial.print("<CR>");
                    }
                #endif
                }
            }

        if (inReceive) {                                            // We're waiting for a command answer
            if ((millis() - startTime) >= gsmTimeout) {
                if (ignoreErrors) {                                 // If errors should be ignored, call next step, if any
                    trace_error_P("Ignoring time out after %d ms, received >%s<, command was %s", millis() - startTime, lastAnswer, lastCommand);
                    if (nextStepCb) {                               // Do we have another callback to execute?
                        (this->*nextStepCb)();                      // Yes, do it
                        return;
                    }
                    setIdle();                                      // No, we just finished.
                    return;
                }
                // Here, we've got a time-out on answer. Should we repeat command?
                if (stepRepeatCount < stepMaxRepeatcount) {
                    // We can repeat the request, do it
                    stepRepeatCount++;                              // Increment repeat count
                    sendCurrentInitStep();                          // Resebd the same command
                    return;
                }
                if (lastAnswer[0]) {
                    trace_error_P("Partial answer: >%s< after %d ms, command was %s", lastAnswer, millis() - startTime, lastCommand);
                    gsmStatus = SIM7000_BAD_ANSWER;
                    restartNeeded = true;
                    restartReason = gsmStatus;
                    setIdle();
                    return;
                } else {                                            // Time-out without any anwser
                    trace_error_P("Timed out after %d ms, received >%s<, command was %s", millis() - startTime, lastAnswer, lastCommand);
                    gsmStatus = SIM7000_TIMEOUT;
                    restartNeeded = true;
                    restartReason = gsmStatus;
                    setIdle();
                    return;
                }
            }
        }

        if (inWaitSmsReady && smsReady) {
            if (debugFlag) trace_debug_P("End of %d ms SMS ready wait, received >%s<", millis() - startTime, lastAnswer);
            inWait = false;
            inWaitSmsReady = false;
            gsmStatus = SIM7000_OK;
            if (nextStepCb) {                                       // Do we have another callback to execute?
                (this->*nextStepCb)();                              // Yes, do it
                return;
            }
                setIdle();                                          // No, we just finished.
            return;
        }

        if (inWait) {
            if ((millis() - startTime) >= gsmTimeout) {
                if (debugFlag) trace_debug_P("End of %d ms wait, received >%s<", millis() - startTime, lastAnswer);
                inWait = false;
                gsmStatus = SIM7000_OK;
                if (nextStepCb) {                                   // Do we have another callback to execute?
                    (this->*nextStepCb)();                          // Yes, do it
                    return;
                }
                    setIdle();                                      // No, we just finished.
                return;
            }
        }
    }
}

/*!

    \brief  Trace some internal variables values (user for debug)

    This routine dumps (almost all) variables using trace_info_P macro (usually defined in Ff_TRACE)

    \param  none
    \return none

*/
void FF_Sim7000::debugState(void) {
    if (traceFlag) enterRoutine(__func__);
    trace_info_P("lastCommand=%s", lastCommand);
    trace_info_P("expectedAnswer=%s", expectedAnswer);
    trace_info_P("lastAnswer=%s", lastAnswer);
    trace_info_P("restartNeeded=%d", restartNeeded);
    trace_info_P("restartReason=%d", restartReason);
    trace_info_P("smsReady=%d", smsReady);
    trace_info_P("gsmIdle=%d", gsmIdle);
    trace_info_P("inReceive=%d", inReceive);
    trace_info_P("gsmTimeout=%d", gsmTimeout);
    trace_info_P("gsmStatus=%d", gsmStatus);
    trace_info_P("index=%d", index);
    trace_info_P("powerStep=%d", powerStep);
    trace_info_P("startTime=%d", millis()-startTime);
    if (powerStepStartTime) {
        trace_info_P("powerStepStartTime=%d", millis()-powerStepStartTime);
    }
    trace_info_P("firstInitDone=%d", firstInitDone);
    trace_info_P("modemSpeaking=%d", modemSpeaking);
    trace_info_P("resetCount=%d", resetCount);
    trace_info_P("restartCount=%d", restartCount);
    trace_info_P("commandCount=%d", commandCount);
    trace_info_P("smsReadCount=%d", smsReadCount);
    trace_info_P("smsForwardedCount=%d", smsForwardedCount);
    trace_info_P("smsSentCount=%d", smsSentCount);
    trace_info_P("Sim7000-debugFlag=%d", debugFlag);
    trace_info_P("Sim7000-traceFlag=%d", traceFlag);
    trace_info_P("Sim7000-traceEnterFlag=%d", traceEnterFlag);
}

/*!

    \brief  Sends an SMS to modem

    This routine pushes an SMS to modem.
        It determines if message is a GMS7 only message or not (in this case, this will be UCS-2)
        If message is GSM7, max length of non chunked SMS is 160. For UCS-2, this is 70.
        When message is longer than these limits, it'll be split in chunks of 153 chars for GSM7, or 67 chars for UCS-2.
        There's a theoretical limit of 255 chunks, but most of operators are limiting in lower size.
        It seems that 7 to 8 messages are accepted by almost everyone, meaning 1200 GSM7 chars, or 550 UCS-2 chars.

    \param[in]  number: phone number to send message to
    \param[in]  text: message to send
    \return none

*/
void FF_Sim7000::sendSMS(const char* number, const char* text) {
    uint16_t utf8Length = strlen(text);                             // Size of UTF-8 message
    uint8_t lengthToAdd;                                            // Length of one UTF-8 char in GSM-7 (or zero if UTF-8 input character outside GSM7 table)
    uint8_t c1;                                                     // First char of UTF-8 message
    uint8_t c2;                                                     // Second char of UTF-8 message
    uint8_t c3;                                                     // Third char of UTF-8 message
    gsm7Length = 0;                                                 // Size of GSM-7 message

    for (uint16_t i = 0; i < utf8Length; i++) {                     // Scan the full message
        c1 = text[i];                                               // Extract first to third chars
        if (i+1 < utf8Length) {c2 = text[i+1];} else {c2 = 0;}
        if (i+2 < utf8Length) {c3 = text[i+2];} else {c3 = 0;}
        lengthToAdd = getGsm7EquivalentLen(c1, c2, c3);             // Get equivalent GSM-7 length
        if (lengthToAdd) {                                          // If char is GSM-7
            gsm7Length += lengthToAdd;                              // Add length
        } else {
            if (debugFlag) trace_info_P("Switched to UTF-8 on char %d (0x%02x) at pos %d", c1, c1, i);
            gsm7Length = 0;                                         // Set length to zero
            break;                                                  // Exit loop
        }
    }

    // Should we split message in chunks?
    if (gsm7Length) {                                               // Is this a GSM-7 message ?
        if (gsm7Length > 160) {                                     // This is a multi-part message
            smsMsgCount = (gsm7Length + 151) / 152;                 // Compute total chunks
            smsMsgId++;
            smsChunkSize = 152;
        } else {
            smsMsgCount = 0;
        }
        if (debugFlag) trace_info_P("gsm7, lenght=%d, msgs=%d", gsm7Length, smsMsgCount);
    } else {                                                        // This is an UCS-2 message
    uint16_t ucs2Length = ucs2MessageLength(text);                  // Get UCS-2 message length
    if (ucs2Length > 70) {                                          // This is a multi-part message
        smsMsgCount = (ucs2Length + 66) / 67;                       // Compute total chunks
        smsMsgId++;
        smsChunkSize = 67;
    } else {
        smsMsgCount = 0;
    }
    if (debugFlag) trace_info_P("ucs2, length=%d, msgs=%d", ucs2Length, smsMsgCount);
    }
    // Save last used number and message
    lastSentNumber = String(number);
    lastSentMessage = String(text);

    // Get local time
    struct tm timeinfo;
    getLocalTime(&timeinfo, 250);
    char dateStr[25];
    strftime(dateStr, sizeof(dateStr), "%Y/%m/%d %H:%M:%S", &timeinfo);
    lastSentDate = String(dateStr);

    // Send first (or only) SMS part
    if (smsMsgCount == 0) {
        sendOneSmsChunk(number, text);
    } else {
        smsMsgIndex = 0;
        uint16_t startPos = smsMsgIndex++ * smsChunkSize;
        sendOneSmsChunk(number, lastSentMessage.substring(startPos, startPos+smsChunkSize).c_str(), smsMsgId, smsMsgCount, smsMsgIndex);    // Send first chunk
    }
}

/*!

    \brief  Sends next SMS chunk to modem

    This routine is called when an SMS chunk has been pushed to modem, to send next part, if it exists

    \param  none
    \return none

*/
void FF_Sim7000::sendNextSmsChunk(void){
    if (smsMsgCount) {                                              // Are we in multi-part message ?
        if (smsMsgIndex < smsMsgCount) {                            // Do we have more chunks to send ?
            uint16_t startPos = smsMsgIndex++ * smsChunkSize;
            sendOneSmsChunk(lastSentNumber.c_str(), lastSentMessage.substring(startPos, startPos+smsChunkSize).c_str(), smsMsgId, smsMsgCount, smsMsgIndex);    // Send first chunk
            return;
        }
    }
    setIdle();                                                      // Message has fully be sent
}

/*!

    \brief  Sends an SMS chunk to modem

    This routine pushes an SMS chunk to modem

    \param[in]  number: phone number to send message to
    \param[in]  text: message to send
    \param[in]  msgId: SMS message identifier (should be incremented for each multi-part message, zero if not multi-part message)
    \param[in]  msgCount: total number of SMS chunks (zero if not multi-part message)
    \param[in]  msgIndex: index of this message chunk (zero if not multi-part message)
    \return none

*/
void FF_Sim7000::sendOneSmsChunk(const char* number, const char* text, const unsigned short msgId, const unsigned char msgCount, const unsigned char msgIndex) {
    if (traceFlag) enterRoutine(__func__);
    char tempBuffer[50];
    int len = smsPdu.encodePDU(number, text, msgId, msgCount, msgIndex);
    if (len < 0)  {
            // -1: OBSOLETE_ERROR
            // -2: UCS2_TOO_LONG
            // -3 GSM7_TOO_LONG
            // -4 MULTIPART_NUMBERS
            // -5 ADDRESS_FORMAT
            // -6 WORK_BUFFER_TOO_SMALL
            // -7 ALPHABET_8BIT_NOT_SUPPORTED
        trace_error_P("Encode error %d sending SMS to %s >%s<", len, number, text);
        return;
    }

    if (debugFlag) trace_debug_P("Sending SMS to %s >%s<", number, text);
    gsmIdle = SIM7000_SEND;
    smsSentCount++;
    snprintf_P(tempBuffer, sizeof(tempBuffer),PSTR("AT+CMGS=%d"), len);
    sendCommand(tempBuffer, &FF_Sim7000::sendSMStext, ">", 10000);
}

/*!

    \brief  Register a SMS received callback routine

    This routine register a callback routine to call when a SMS message is received

    Callback routine will be called with 4 parameters:
        (char*) number: phone number of SMS sender
        (char*) date: date of received SMS (as delivered by the network)
        (char*) message: received message in UTF-8 encoding

    \param[in]  routine to call when a SMS is received
    \return none

*/
void FF_Sim7000::registerSmsCb(void (*readSmsCallback)(const char* __number, const char* __date, const char* __message)) {
    if (traceFlag) enterRoutine(__func__);
    readSmsCb = readSmsCallback;
}

/*!

    \brief  Register a SMS send callback routine

    This routine register a callback routine to call when a SMS message is sent

    Callback routine will be called with 4 parameters:
        (char*) number: phone number of SMS receiver
        (char*) date: date of sent SMS
        (char*) message: sent message in UTF-8 encoding

    \param[in]  routine to call when a SMS is sent
    \return none

*/
void FF_Sim7000::registerSendCb(void (*sendSmsCallback)(const char* __number, const char* __date, const char* __message)) {
    if (traceFlag) enterRoutine(__func__);
    sendSmsCb = sendSmsCallback;
}

/*!

    \brief  Register an answer received callback routine

    This routine register a callback routine to call when an answer is received from modem

    Callback routine will be called with 1 parameter:
        (char*) answer: received answer

    \param[in]  routine to call when a SMS is received
    \return none

*/
void FF_Sim7000::registerLineCb(void (*recvLineCallback)(const char* __answer)) {
    if (traceFlag) enterRoutine(__func__);
    recvLineCb = recvLineCallback;
}

/*!

    \brief  Send next init step  message

    This routine send next message during modem initialization phase
    \return none

*/
void FF_Sim7000::sendNextInitStep(void){
    if (traceFlag) enterRoutine(__func__);
    stepPtr++;
    if (stepPtr < STEP_SIZE) {
        sendCurrentInitStep();
    } else {
        initComplete();
    }
}

/*!

    \brief  Send current init step  message

    This routine send current message during modem initialization phase

    \return none

*/
void FF_Sim7000::sendCurrentInitStep(void){
    if (traceFlag) enterRoutine(__func__);
    // check for stepPtr into table
    if (stepPtr < STEP_SIZE) {
        // Do we have a routine to run ?
        if (initSteps[stepPtr].nextStep) {
            trace_debug_P("sendCurrentInitStep %d - Call routine", stepPtr);
            (this->*initSteps[stepPtr].nextStep)();
        } else {
            // No routine, send data
            if (*initSteps[stepPtr].waitFor) {
                    // We have a specific data to wait for
                trace_debug_P("sendCurrentInitStep %d - Send %s, wait for %s, timeout %d, repeat %d", stepPtr, initSteps[stepPtr].command, initSteps[stepPtr].waitFor, initSteps[stepPtr].timeout, initSteps[stepPtr].repeat);
                sendCommand(initSteps[stepPtr].command, &FF_Sim7000::sendNextInitStep, initSteps[stepPtr].waitFor, initSteps[stepPtr].timeout), initSteps[stepPtr].repeat;
            } else {
                // Use default answer
                trace_debug_P("sendCurrentInitStep %d - Send %s, wait for %s, timeout %d, repeat %d", stepPtr, initSteps[stepPtr].command, DEFAULT_ANSWER, initSteps[stepPtr].timeout, initSteps[stepPtr].repeat);
                sendCommand(initSteps[stepPtr].command, &FF_Sim7000::sendNextInitStep, DEFAULT_ANSWER, initSteps[stepPtr].timeout, initSteps[stepPtr].repeat);
            }
        }
    } else {
        trace_error_P("Trying to execute step %d, max is %d", stepPtr, sizeof(initSteps));
    }
}

/*!

    \brief  Delete SMS from the storage area

    This routine delete (some) SMS from storage using AT+CMGD command
    \param[in]  index as used by AT+CMGD
    \param[in]  flag as used by AT+CMGD
    \return nonex

*/
void FF_Sim7000::deleteSMS(int index, int flag) {
    if (traceFlag) enterRoutine(__func__);
    char tempBuffer[50];

    snprintf_P(tempBuffer, sizeof(tempBuffer), PSTR("AT+CMGD=%d,%d"), index, flag);
    sendCommand(tempBuffer, &FF_Sim7000::setIdle, DEFAULT_ANSWER, 20000);   // Wait up to 20 seconds for OK
}

/*!

    \brief  Issue an AT command (to be used as debug as no command tracking is done)

    This routine will send an out of band AT command to modem.

    It could be used to debug/test modem.

    Modem answer is ignored and discarded.

    \param[in]  AT command to be send
    \return none

*/
void FF_Sim7000::sendAT(const char *command) {
    if (traceFlag) enterRoutine(__func__);
    sendCommand(command);
    inReceive = false;
}

/*!

    \brief  Issue an EOF command (to be used as debug as no command tracking is done)

    This routine will send an EOF command to modem.

    It could be used to debug/test modem.

    Modem answer is ignored and discarded.

    \param  none
    \return none

*/
void FF_Sim7000::sendEOF(void) {
    if (traceFlag) enterRoutine(__func__);
    sendCommand(0x1a);
    inReceive = false;
}

/*!

    \brief  Check if modem should be restarted

    This routine check if modem restart has been asked by code.

    \param  none
    \return true if modem should be restarted, false else

*/
bool FF_Sim7000::needRestart(void){
    if (traceFlag) enterRoutine(__func__);
    return restartNeeded;
}

/*!

    \brief  Set the restart flag

    This routine sets the restart required flag to a given value

    \param[in]  value to be set (true or false)
    \return none

*/
void FF_Sim7000::setRestart(bool restartFlag){
    if (traceFlag) enterRoutine(__func__);
    restartNeeded = restartFlag;
}

/*!

    \brief  Checks if modem is idle

    This routine checks if modem is idle (not initializing, sending nor receiving)

    \param  none
    \return true if modem is idle, false else

*/
bool FF_Sim7000::isIdle(void){
    if (traceFlag) enterRoutine(__func__);
    return (gsmIdle == SIM7000_IDLE);
}

/*!

    \brief  Checks if modem is sending something

    This routine checks if modem is sending something (either commands or messages)

    \param  none
    \return true is modem is sending, false else

*/
bool FF_Sim7000::isSending(void){
    if (traceFlag) enterRoutine(__func__);
    return (gsmIdle == SIM7000_SEND);
}

/*!

    \brief  Checks if modem is receiving

    This routine checks if modem is receiving a SMS

    \param  none
    \return true if a SMS is receiving, false else

*/
bool FF_Sim7000::isReceiving(void){
    if (traceFlag) enterRoutine(__func__);
    return (gsmIdle == SIM7000_RECV);
}

/*!

    \brief  [Private] Modem initialization: open modem at a given speed

    \param[in]  baudRate: speed (in bds) to use to open modem
    \return none

*/
void FF_Sim7000::openModem(long baudRate) {
    if (traceFlag) enterRoutine(__func__);
    #ifdef FF_SIM7000_USE_SOFTSERIAL
        if (debugFlag) trace_debug_P("Opening modem at %d bds, rx=%d, tx=%d", baudRate, modemTxPin, modemRxPin);
        // Open modem at given speed
        Sim7000Serial.begin(baudRate, SWSERIAL_8N1, modemTxPin, modemRxPin, false, MAX_ANSWER + 3); // Connect to Serial Software
        // Enable TX interruption for speeds up to 19200 bds
        Sim7000Serial.enableIntTx((baudRate <= 19200));
    #else
        #if !defined(FF_SIM7000_USE_SERIAL1) && !defined(FF_SIM7000_USE_SERIAL2)
            if (debugFlag) trace_debug_P("Opening modem at %d bds", baudRate);
            Sim7000Serial.begin(baudRate, SERIAL_8N1);
            // We're on Serial, disable debug output to be able to swap Serial to D8/D7
            #ifndef ESP32
                Sim7000Serial.setDebugOutput(false);
                Sim7000Serial.swap();
            #endif
        #else
            #ifdef ESP32
                if (debugFlag) trace_debug_P("Opening modem at %d bds, rx=%d, tx=%d", baudRate, modemRxPin, modemTxPin);
                Sim7000Serial.begin(baudRate, SERIAL_8N1, modemRxPin, modemTxPin);
            #endif
            #ifdef ESP8266
                #error ESP8266 can only use Serial and SoftwareSerial, not SERIAL1 or Serial2!
            #endif
        #endif
    #endif
    // Flush pending input chars
    while (Sim7000Serial.available()) {
        Sim7000Serial.read();
    }
    smsReady = false;
}

/*!

    \brief  Return restart reason

    \param  none
    \return restart reason

*/
int FF_Sim7000::getRestartReason(void) {
    return restartReason;
}

/*!

    \brief  [Private] Modem initialization: wait to receive SMS ready for 30 seconds

    \param  none
    \return none

*/
void FF_Sim7000::waitUntilSmsReady(void) {
    if (traceFlag) enterRoutine(__func__);
    if (!smsReady) {
        waitSmsReady(30000, &FF_Sim7000::sendNextInitStep);
    } else {
        if (debugFlag) trace_debug_P("SMS ready already received", NULL);
        sendNextInitStep();
    }
}

/*!

    \brief  [Private] Modem initialization: extract SCA number and give it to PDU class

    \param  none
    \return none

*/
void FF_Sim7000::gotSca(void) {
    if (traceFlag) enterRoutine(__func__);
    // Extract SCA from message
    char* ptrStart;
    char scaNumber[MAX_SMS_NUMBER_LEN];

    // Parse the response if it contains a valid CSCA_INDICATOR
    ptrStart = strstr(lastAnswer, CSCA_INDICATOR);

    if (ptrStart == NULL) {
        trace_error_P("Can't find %s in %s",CSCA_INDICATOR, lastAnswer);
        restartReason = SIM7000_BAD_ANSWER;
        restartNeeded = true;
        return;
    }
    //  First token is +CSCA
    char* token = strtok (ptrStart, "\"");
    if (token == NULL) {
        trace_error_P("Can't find first token in %s", lastAnswer);
        restartReason = SIM7000_BAD_ANSWER;
        restartNeeded = true;
        return;
    }

    // Second token is is SCA number
    token = strtok (NULL, "\"");
    if (token == NULL) {
        trace_error_P("Can't find second token in %s", lastAnswer);
        restartReason = SIM7000_BAD_ANSWER;
        restartNeeded = true;
        return;
    }
    strncpy(scaNumber, token, sizeof(scaNumber));
    // Check SCA number (first char can be "+", all other should be digit)
    for (int i = 0; scaNumber[i]; i++) {
        // Is char not a number?
        if (scaNumber[i] < '0' || scaNumber[i] > '9') {
            // First char could be '+'
            if (scaNumber[0] != '+' || i != 0) {
                trace_error_P("Bad SCA number %s at %d", scaNumber, i+1);
                restartReason = SIM7000_BAD_ANSWER;
                restartNeeded = true;
                return;
            }
        }
    }
    if (debugFlag) trace_debug_P("setting SCA to %s", scaNumber);
    smsPdu.setSCAnumber(scaNumber);
    resetLastAnswer();
    sendNextInitStep();
}

/*!

    \brief  [Private] Modem initialization: end of initialization

    \param  none
    \return none

*/
void FF_Sim7000::initComplete(void) {
    if (traceFlag) enterRoutine(__func__);
    if (gsmStatus) {
        restartNeeded = true;
        restartReason = gsmStatus;
    } else {
        setIdle();
        trace_info_P("SMS gateway started, restart count = %d", restartCount);
        restartCount++;
    }
}

/*!

    \brief  [Private] Send a PDU containing a SMS

    \param  none
    \return none

*/
void FF_Sim7000::sendSMStext(void) {
    if (traceFlag) enterRoutine(__func__);

    if (debugFlag) trace_debug_P("Message: %s", smsPdu.getSMS());
    Sim7000Serial.write(smsPdu.getSMS());
    sendCommand(0x1a, &FF_Sim7000::sendNextSmsChunk, "+CMGS:", 60000);
}

/*!

    \brief  [Private] Wait for SMS ready message a given time

    \param[in]  waitMs: Time (ms) to wait for SMS ready message
    \param[in]  nextStep: Routine to call as next step in sequence
    \return none

*/
void FF_Sim7000::waitSmsReady(unsigned long waitMs, void (FF_Sim7000::*nextStep)(void)) {
    if (traceFlag) enterRoutine(__func__);
    if (debugFlag) trace_debug_P("Waiting SMS Ready for %d ms", waitMs);
    gsmTimeout = waitMs;
    gsmStatus = SIM7000_RUNNING;
    nextStepCb = nextStep;
    startTime = millis();
    inReceive = false;
    inWait = true;
    inWaitSmsReady = true;
}

/*!

    \brief  [Private] Sends a (char*) command

    \param[in]  command: Command to send (char*). If empty, will wait for answer of a previously sent command
    \param[in]  nextStep: Routine to call as next step in sequence
    \param[in]  resp: Expected command answer
    \param[in]  cdeTimeout: Maximum time (ms) to wait for correct answer
    \return none

*/
void FF_Sim7000::sendCommand(const char *command, void (FF_Sim7000::*nextStep)(void), const char *resp, unsigned long cdeTimeout, uint8_t repeat) {
    if (traceFlag) enterRoutine(__func__);
    commandCount++;
    gsmTimeout = cdeTimeout;
    gsmStatus = SIM7000_RUNNING;
    nextStepCb = nextStep;
    stepMaxRepeatcount = repeat;
    strncpy(expectedAnswer, resp, sizeof(expectedAnswer));
    if (debugFlag) trace_debug_P("Issuing command: %s", command);
    // Send command if defined (else, we'll just wait for answer of a previously sent command)
    if (command[0]) {
        // Is this a new command?
        if (strcmp(lastCommand, command)) {
            // Yes, reset repeat count
            stepRepeatCount = 0;
        }
        strncpy(lastCommand, command, sizeof(lastCommand));         // Save last command
        resetLastAnswer();
        Sim7000Serial.write(command);
        Sim7000Serial.write('\r');
    }
    startTime = millis();
    inReceive = true;
    inWait = false;
    inWaitSmsReady = false;
    nextLineIsSmsMessage = false;
}

/*!

    \brief  [Private] Sends a (uint8_t) command

    \param[in]  command: Command to send (uint8_t)
    \param[in]  nextStep: Routine to call as next step in sequence
    \param[in]  resp: Expected command answer
    \param[in]  cdeTimeout: Maximum time (ms) to wait for correct answer
    \return none

*/
void FF_Sim7000::sendCommand(const uint8_t command, void (FF_Sim7000::*nextStep)(void), const char *resp, unsigned long cdeTimeout) {
    if (traceFlag) enterRoutine(__func__);
    commandCount++;
    gsmTimeout = cdeTimeout;
    gsmStatus = SIM7000_RUNNING;
    nextStepCb = nextStep;
    strncpy(expectedAnswer, resp, sizeof(expectedAnswer));
    resetLastAnswer();
    if (debugFlag) trace_debug_P("Issuing command: 0x%x", command);
    Sim7000Serial.write(command);
    startTime = millis();
    inReceive = true;
    inWaitSmsReady = false;
}

/*!

    \brief  [Private] Set modem idle

    \param  none
    \return none

*/
void FF_Sim7000::setIdle(void) {
    if (traceFlag) enterRoutine(__func__);
    trace_debug_P("Modem is idle", NULL);
    gsmIdle = SIM7000_IDLE;
    inReceive = false;
    resetLastAnswer();
}

/*!

    \brief  [Private] Debug: trace each entered routine

    By default, do nothing as very verbose. Enable it only when really needed.

    \param[in]  routine name to display
    \return none

*/
void FF_Sim7000::enterRoutine(const char* routineName) {
    if (traceEnterFlag) trace_debug_P("Entering %s", routineName);
}

/*!

    \brief  [Private] Read a SMS header

    \param[in]  msg: modem answer
    \return none

*/
void FF_Sim7000::readSmsHeader(const char* msg) {
    if (traceFlag) enterRoutine(__func__);
    index = 0;

    // Answer format is:
    // +CMT ,33
    // 07913396050066F0040B913306672146F00000328041102270800FCDF27C1E3E9741E432885E9ED301

    // Parse the response if it contains a valid SMS_INDICATOR
    char* ptrStart = strstr(msg, SMS_INDICATOR);
    if (ptrStart == NULL) {
        trace_error_P("Can't find %s in %s", SMS_INDICATOR, msg);
        return;
    }
    if (debugFlag) trace_debug_P("Waiting for SMS", NULL);
    nextLineIsSmsMessage = true;
}

/*!

    \brief  [Private] Read a PDU containing a received SMS

    \param[in]  msg: received PDU
    \return none

*/
void FF_Sim7000::readSmsMessage(const char* msg) {
    if (traceFlag) enterRoutine(__func__);
    if (smsPdu.decodePDU(msg)) {
        if (smsPdu.getOverflow()) {
            trace_warn_P("SMS decode overflow, partial message only", NULL);
        }
        lastReceivedNumber = String(smsPdu.getSender());
        lastReceivedDate = String(smsPdu.getTimeStamp());
        lastReceivedMessage = String(smsPdu.getText());
        smsForwardedCount++;
        if (debugFlag) trace_debug_P("Got SMS from %s, sent at %s, >%s<", lastReceivedNumber.c_str(), lastReceivedDate.c_str(), lastReceivedMessage.c_str());
        if (readSmsCb) (*readSmsCb)(lastReceivedNumber.c_str(), lastReceivedDate.c_str(), lastReceivedMessage.c_str());
    } else {
        trace_error_P("SMS PDU decode failed", NULL);
    }
    deleteSMS(1,2);
}

/*!

    \brief  [Private] Clean ast answer

    \param  none
    \return none

*/
void FF_Sim7000::resetLastAnswer(void) {
    if (traceFlag) enterRoutine(__func__);
    memset(lastAnswer, 0, sizeof(lastAnswer));
}

/*!

    \brief  Return GSM7 equivalent length of one UTF-8 character

    This routine takes one UTF-8 character coded on up-to 3 bytes to return it's length when coded in GSM7

    \param[in]  c1: first byte of UTF-8 character to analyze
    \param[in]  c2: second byte of UTF-8 character to analyze (or zero if end of message)
    \param[in]  c3: third byte of UTF-8 character to analyze (or zero if end of message)
    \return Length of character when coded in GSM7 (or zero if UTF-8 input character outside GSM7 table)

*/

uint8_t FF_Sim7000::getGsm7EquivalentLen(const uint8_t c1, const uint8_t c2, const uint8_t c3) {
    if (
            // These are one byte UTF8 char coded on one byte GSM7 char
            (c1 == 0x0a)    /* Linefeed */                                                                                      ||
            (c1 == 0x0d)    /* Carriage return */                                                                               ||
            (c1 >= 0x20     /* Space */                                 && c1 <= 0x5a) /* Capital letter Z */                   ||
            (c1 == 0x5f)    /* Underscore */                                                                                    ||
            (c1 >= 0x61     /* Small letter a */                        && c1 <= 0x7a) /* Small letter z */
        ) {
            return 1;
    }
    if ((c1 == 0xc2) && (
            // These are two bytes UTF8 char coded on one byte GSM7 char
            (c2 == 0xa1)    /* Inverted exclamation mark */                                                                     ||
            (c2 >= 0xa3     /* Pound sign */                            && c2 <= 0xa5) /* Yuan/Yen sign */                      ||
            (c2 == 0xa7)    /* Section sign */                                                                                  ||
            (c2 == 0xbf)    /* Inverted question mark */
        )) {
            return 1;
    }
    if ((c1 == 0xc3) && (
            // These are two bytes UTF8 char coded on one byte GSM7 char
            (c2 >= 0x84     /* Capital letter A with diaeresis */       && c2 <= 0x87) /* Capital letter C with cedilla */      ||
            (c2 == 0x89)    /* Capital letter E with acute accent */                                                            ||
            (c2 == 0x91)    /* Capital letter N with tilde */                                                                   ||
            (c2 == 0x96)    /* Capital letter O with diaeresis */                                                               ||
            (c2 == 0x98)    /* Capital letter O with stroke */                                                                  ||
            (c2 == 0x9c)    /* Capital letter U with diaeresis */                                                               ||
            (c2 >= 0x9f     /* Small letter German Eszett */            && c2 <= 0xa0) /* Small letter a with grave accent */   ||
            (c2 >= 0xa4     /* Small letter a with diaeresis */         && c2 <= 0xa6) /* Small letter ae */                    ||
            (c2 >= 0xa8     /* Small letter e with grave accent */      && c2 <= 0xa9) /* Small letter e with acute accent */   ||
            (c2 == 0xac)    /* Small letter i with grave accent */                                                              ||
            (c2 >= 0xb1     /* Small letter n with tilde */             && c2 <= 0xb2) /* Small letter o with grave accent */   ||
            (c2 == 0xb6)    /* Small letter o with diaeresis */                                                                 ||
            (c2 >= 0xb8     /* Small letter o with stroke */            && c2 <= 0xb9) /* Small letter u with grave accent */   ||
            (c2 == 0xbc)    /* Small letter u with diaeresis */
        )) {
            return 1;
    }
    if (
            // These are one byte UTF8 char coded on two bytes GSM7 char
            (c1 == 0x0c)    /* Form feed */ ||
            (c1 >= 0x5b     /* Left square bracket */                   && c1 <= 0x5e) /* Caret / Circumflex */                 ||
            (c1 >= 0x7b     /* Left curly bracket */                    && c1 <= 0x7e) /* Tilde */
        ) {
            return 2;
    }
    if (c1 == 0xe2 &&
            // This is three bytes UTF8 char coded on two bytes GSM7 char
            c2 == 0x82 &&
            c3 == 0xac      /* Euro sign */
        ) {
        return 2;
    }
    return 0;
}

/*!

    \brief  Return UCS-2 equivalent length of one UTF-8 character

    This routine takes one UTF-8 message to return it's length when coded in UCS-2

    \param[in]  text: message to be scanned
    \return Length of character when coded in UCS-2

*/

uint16_t FF_Sim7000::ucs2MessageLength(const char* text) {
    uint16_t utf8CharCount = 0;

    // Get UTF-8 message length
    while (*text) {
        // Cout only first byte of each UTF-8 sequence
        utf8CharCount += (*text++ & 0xc0) != 0x80;
    }
    // UCS-2 if 2 chars for each UTF-8 character
    return utf8CharCount * 2;
}