# FF_SIM7000
Fully asynchronous SMS send/receive in PDU mode for SIM7xxx modem class

## What's for?
This class allows asynchronously sending/receiving SMS using a SIM7xxx modem using PDU mode.

Messages are in UTF-8 format and automatically converted into GSM7 or UCS2, and split in multiple messages if needed.

A callback routine in your program will be called each time a SMS is received.

You also may send SMS directly.

By default, logging/debugging is done through FF_TRACE macros, allowing to easily change code.

You may have a look at https://github.com/FlyingDomotic/FF_SmsServer32 which shows how to use it

## Prerequisites

Can be used directly with Arduino IDE or PlatformIO.

## Installation

Clone repository somewhere on your disk.
```
cd <where_you_want_to_install_it>
git clone https://github.com/FlyingDomotic/FF_SIM7000.git FF_SIM7000
```

Note that <where_you_want_to_install_it> could astuciously be your compiler libraries folder ;-)

## Update

Go to code folder and pull new version:
```
cd <where_you_installed_FF_SIM7000>
git pull
```

Note: if you did any changes to files and `git pull` command doesn't work for you anymore, you could stash all local changes using:
```
git stash
```
or
```
git checkout <modified file>
```

## Documentation

Documentation could be built using doxygen, either using makedoc.sh (for Linux) or makedoc.bat (for Windows), or running
```
doxygen doxyfile
```

HTML and RTF versions will then be available in `documentation` folder.
