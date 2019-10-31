/*
 * Sc_Serial_Library_new.h
 *
 *  Created on: 2017年6月5日
 *      Author: msi
 */

#ifndef SERIAL_LIB_NEW_SC_SERIAL_LIBRARY_NEW_H_
#define SERIAL_LIB_NEW_SC_SERIAL_LIBRARY_NEW_H_

#include "../cg_src/r_cg_macrodriver.h"
#include "../cg_src/r_cg_sci.h"
#include "stdarg.h"
#include "stdio.h"

#define SERIAL_LIBRARY_BUFFER_SIZE 50

void Bluetooth_Writestring(uint8_t string[]);
int my_printf(const char *fmt, ...);
void mavlink_printf(const char *fmt, ...);

#endif /* SERIAL_LIB_NEW_SC_SERIAL_LIBRARY_NEW_H_ */
