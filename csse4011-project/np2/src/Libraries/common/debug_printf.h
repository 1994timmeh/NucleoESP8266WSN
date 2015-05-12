/*
 *  linux/lib/vsprintf.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 */

/* vsprintf.c -- Lars Wirzenius & Linus Torvalds. */

#ifndef __PRINTF_SERIAL_H__
#define __PRINTF_SERIAL_H__

#define PRINTF_DELAY 0x04FF00

extern void debug_putc(char c);
extern void debug_flush();
extern char debug_getc();
extern void debug_printf (const char *fmt, ...);
extern void hex_dump (const unsigned char *buf, unsigned int addr, unsigned int len);
//extern void vDebugSendHook (char data);

#endif/*__PRINTF_SERIAL_H__*/
