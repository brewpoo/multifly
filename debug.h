//
//  debug.h
//  Project
//
//  Created by Jon Lochner on 11/11/14.
//
//

#include "lib_serial.h"


#ifdef DEBUG
#define DEBUG_INIT(baud) lib_serial_initport(USBPORTNUMBER,baud);
#define DEBUG_PRINT(str) lib_serial_sendstring(USBPORTNUMBER,str);
#else
#define DEBUG_INIT(baud) ((void)0);
#define DEBUG_PRINT(str) ((void)0);
#endif

