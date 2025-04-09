/*!
    \file
    \brief  Implements a fully asynchronous SMS send/receive in PDU mode for Sim7000 modem class
    \author Flying Domotic
    \date   March 31st, 2025

    Have a look at FF_Sim7000.cpp for details

*/

#ifndef FF_Sim7000_h
#define FF_Sim7000_h

#include <Arduino.h>

// Constants
#define SIM7000_CMD_TIMEOUT 4000                                    //!< Standard AT command timeout (ms)
#define MAX_SMS_NUMBER_LEN 20                                       //!< SMS number max length
#define MAX_ANSWER 500                                              //!< AT command answer max length
#define DEFAULT_ANSWER "OK"                                         //!< AT command default answer
#define CREG_MSG "+CREG: "                                          //!< CREG unsolicited message
#define CREG_QUERY "+CREG?"                                         //!< CREG request
#define SMS_INDICATOR "+CMT: "                                      //!< SMS received indicator
#define CSCA_INDICATOR "+CSCA:"                                     //!< SCA value indicator
#define GSM_TIME "*PSUTTZ: "                                        //!< GSM network time
//#define SIM7000_KEEP_CR_LF                                        //!< Keep CR & LF in displayed messages (by default, they're replaced by ".")

// Enums
#define SIM7000_OK 0
#define SIM7000_RUNNING 1
#define SIM7000_TIMEOUT 1
#define SIM7000_TOO_LONG 2
#define SIM7000_BAD_ANSWER 3
#define SIM7000_CM_ERROR 4
#define SIM7000_NEED_INIT 5

#define SIM7000_IDLE 0
#define SIM7000_SEND 1
#define SIM7000_RECV 2
#define SIM7000_STARTING 3
#define SIM7000_NOT_CONNECTED 4

#ifndef SIM7000_PIN_ACTIVE
    #define SIM7000_PIN_ACTIVE HIGH
#endif
#ifndef SIM7000_PIN_INACTIVE
    #define SIM7000_PIN_INACTIVE LOW
#endif

// Class definition
class FF_Sim7000 {
public:
    // public class
    /*! \class FF_Sim7000
        \brief Implements a fully asynchronous SMS send/receive in PDU mode for Sim7000 modem class

        This class allows asynchronously sending/receiving SMS using an Sim7000 (and probably others) modem using PDU mode.

        Messages are in UTF-8 format and automatically converted into GSM7 (160 characters) or UCS-2 (70 characters).

        A callback routine in your program will be called each time a SMS is received.

        You also may send SMS directly.

        By default, logging/debugging is done through FF_TRACE macros, allowing to easily change code.

        It may also be used with FF_WebServer class, as containing routines to map with it.

        You may have a look at https:                               //github.com/FlyingDomotic/FF_SmsServer which shows how to use it

        By default, FF_Sim7000 uses Sim7000 modem connected on D8(TX) and D7(RX) on ESP8266.

        You can use #define USE_DIRECT_CONNECTIONS_FOR_SIM7000 if Sim7000 is connected directly to TX/RX. Don't forget to set Serial.setDebugOutput to avoid garbage.

        You may also use #define USE_SOFTSERIAL_FOR_SIM7000 to use SoftwareSerial instead of Serila.swap(), but
            be aware that program will crash if you're using any asynchronous libraries (like espAsyncxxx).

    */
    FF_Sim7000();

    // Public routines (documented in FF_Sim7000.cpp)
    void begin(long baudRate, int8_t rxPin, int8_t txPin, int8_t powerPin=-1);
    void doLoop(void);
    void debugState(void);
    void sendSMS(const char* number, const char* text);
    void sendOneSmsChunk(const char* number, const char* text, const unsigned short msgId = 0, const unsigned char msgCount = 0, const unsigned char msgIndex = 0);
    void registerSmsCb(void (*readSmsCallback)(const char* __number, const char* __date, const char* __message));
    void registerSendCb(void (*sendSmsCallback)(const char* __number, const char* __date, const char* __message));
    void registerLineCb(void (*recvLineCallback)(const char* __answer));
    void deleteSMS(int index, int flag);
    void sendAT(const char* command);
    void sendEOF(void);
    void setPowerPin(void);
    bool needRestart(void);
    int getRestartReason(void);
    void setRestart(bool restartFlag);
    void waitUntilSmsReady(void);
    void gotSca(void);
    void initComplete(void);
    bool isIdle(void);
    bool isSending(void);
    bool isReceiving(void);
    uint8_t getGsm7EquivalentLen(const uint8_t c1, const uint8_t c2, const uint8_t c3);
    uint16_t ucs2MessageLength(const char* text);

    // Public variables
    bool debugFlag;                                                 //!< Show debug messages flag
    bool traceFlag;                                                 //!< Show trace messages flag
    bool traceEnterFlag;                                            //!< Show each routine entering flag
    bool ignoreErrors;                                              //!< Ignore errors flag
    bool smsReady;                                                  //!< True if "SMS ready" seen
    String lastReceivedNumber;                                      //!< Phone number of last received SMS
    String lastReceivedDate;                                        //!< Date of last received SMS
    String lastReceivedMessage;                                     //!< Message of last received SMS
    String lastSentNumber;                                          //!< Phone number of last SMS sent
    String lastSentDate;                                            //!< Date of last SMS sent
    String lastSentMessage;                                         //!< Message of last SMS sent

private:
    // Private routines (documented in FF_Sim7000.cpp)
    void open(void);
    void sendCommand(const char *command, void (FF_Sim7000::*nextStep)(void)=NULL, const char *resp=DEFAULT_ANSWER, unsigned long cdeTimeout=SIM7000_CMD_TIMEOUT, uint8_t repeat=0);
    void sendCommand(const uint8_t command, void (FF_Sim7000::*nextStep)(void)=NULL, const char *resp=DEFAULT_ANSWER, unsigned long cdeTimeout=SIM7000_CMD_TIMEOUT);
    void waitSmsReady(unsigned long waitMs, void (FF_Sim7000::*nextStep)(void)=NULL);
    void sendNextSmsChunk(void);
    void openModem(long baudRate);
    void sendSMStext(void);
    void setIdle(void);
    void enterRoutine(const char* routineName);
    void readSmsHeader(const char* msg);
    void readSmsMessage(const char* msg);
    void resetLastAnswer(void);

    // Private variables
    unsigned long startTime;                                        //!< Last command start time
    unsigned int commandCount;                                      //!< Count of commands sent
    unsigned int resetCount;                                        //!< Count of GSM reset
    unsigned int restartCount;                                      //!< Count of successful GSM restart
    unsigned int smsReadCount;                                      //!< Count of SMS read
    unsigned int smsForwardedCount;                                 //!< Count of SMS analyzed
    unsigned int smsSentCount;                                      //!< Count of SMS sent
    int8_t modemRxPin;                                              //!< Modem RX pin
    int8_t modemTxPin;                                              //!< Modem TX pin
    int8_t modemPowerPin;                                           //!< Modem power pin
    long modemSpeed;                                                //!< Modem speed
    int8_t powerStep;                                               //!< Modem power step index
    bool modemConnected;                                            //!< Modem connected to GSM network flag
    void (FF_Sim7000::*nextStepCb)(void);                           //!< Callback for next step in command execution
    void (*readSmsCb)(const char* __number, const char* __date, const char* __message); //!< Callback for readSMS
    void (*sendSmsCb)(const char* __number, const char* __date, const char* __message); //!< Callback for sendSMS
    void (*recvLineCb)(const char* __answer);                       //!< Callback for received line
    void sendNextInitStep(void);                                    //!< Send next init step command
    void sendCurrentInitStep(void);                                 //!< Send current init step command
    int index;                                                      //!< Index of last read SMS
    int restartReason;                                              //!< Last restart reason
    unsigned long gsmTimeout;                                       //!< Timeout value (ms)
    unsigned long powerStepStartTime;                               //!< Last power step start
    unsigned long powerStepDuration[5];                             //!< Power steps duration
    int gsmStatus;                                                  //!< Last command status
    int gsmIdle;                                                    //!< GSM is idle flag
    bool inReceive;                                                 //!< Are we receiving answer?
    bool inWait;                                                    //!< Are we waiting for some time?
    bool inWaitSmsReady;                                            //!< Are we waiting for SMS Ready?
    bool restartNeeded;                                             //!< Restart needed flag
    bool nextLineIsSmsMessage;                                      //!< True if next line will be an SMS message (just after SMS header)
    bool firstInitDone;                                             //!< True if first init done
    bool modemSpeaking;                                             //!< Modem has spoke
    char lastAnswer[MAX_ANSWER];                                    //!< Contains the last GSM command anwser
    char expectedAnswer[10];                                        //!< Expected answer to consider command ended
    char lastCommand[30];                                           //!< Last command sent
    uint16_t gsm7Length;                                            //!< Size of GSM-7 message (0 for UCS-2 messages)
    unsigned short smsMsgId;                                        //!< Multi-part message ID (to be incremented for each multi-part message sent)
    uint8_t smsMsgIndex;                                            //!< Chunk index of current multi-part message
    uint8_t smsMsgCount;                                            //!< Chunk total count of current multi-part message
    uint8_t smsChunkSize;                                           //!< Chunk size for this message
};
#endif