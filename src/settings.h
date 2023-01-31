
#ifndef SETTINGS_H

#define SETTINGS_H

#include "network_config.h"
// #include "ezTime.h" //ntp and other functions

#endif

#define DEBUGLOG

#ifdef DEBUGLOG

#define DEBUGPRINT(x) Serial.print(x)
#define DEBUGPRINTDEC(x) Serial.print(x, DEC)
#define DEBUGPRINTLN(x) Serial.println(x)

#else

#define DEBUGPRINT(x)
#define DEBUGPRINTDEC(x)
#define DEBUGPRINTLN(x)

#endif
