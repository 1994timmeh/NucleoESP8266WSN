/*
 *  linux/lib/vsprintf.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 *  Modified by MDS and J. Yates (2014)
 */

/* vsprintf.c -- Lars Wirzenius & Linus Torvalds. */

//#include <FreeRTOS.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>

#include "board.h"
#include "stm32f4xx_hal_conf.h"


#include "debug_printf.h"


#ifdef ENABLE_VCP
	#include "usbd_cdc_vcp.h" 
	#define DEBUG_SEND_HOOK(c)	{BRD_debuguart_putc(c);} // {VCP_putc(c);}	//Calls both UART and VCP putc
	#define DEBUG_SENDCHAR_HOOK(c)	{BRD_debuguart_putc(c);} // {VCP_putc(c); }	//Calls both UART and VCP putc
	#define DEBUG_FLUSH_HOOK()	{VCP_txflush();}
	#define DEBUG_FLUSH_DELAY()	{debug_delay(PRINTF_DELAY);}


#elif ENABLE_DEBUG_UART
	#define DEBUG_SEND_HOOK(c)	{BRD_debuguart_putc(c);}	//Calls only UART putc
	#define DEBUG_SENDCHAR_HOOK(c)	{BRD_debuguart_putc(c);}
	#define DEBUG_FLUSH_HOOK()
	#define DEBUG_FLUSH_DELAY()

#else
	#define DEBUG_SEND_HOOK(c)
	#define DEBUG_SENDCHAR_HOOK(c)
	#define DEBUG_FLUSH_HOOK()
	#define DEBUG_FLUSH_DELAY()

#endif


#ifdef ENABLE_VCP
// Delay function for VCP
void debug_delay(__IO unsigned long nCount) {
  	while(nCount--) {
  	}
}
#endif

void debug_putc(char c) {
	DEBUG_SENDCHAR_HOOK(c);
}

void debug_flush() {
	DEBUG_FLUSH_HOOK();


}

char debug_getc(void) {
	uint8_t c = '\0';

#ifdef ENABLE_VCP
	VCP_getc(&c);
#endif

	//Check if character is received is not NULL from VCP
	if (c == '\0') {
		c = BRD_debuguart_getc();
	}

	return (char) c; 
}




unsigned long
simple_strtoul (const char *cp, char **endp, unsigned int base)
{
  unsigned long result = 0, value;

  if (*cp == '0')
    {
      cp++;
      if ((*cp == 'x') && isxdigit ((int)cp[1]))
	{
	  base = 16;
	  cp++;
	}
      if (!base)
	{
	  base = 8;
	}
    }
  if (!base)
    {
      base = 10;
    }
  while (isxdigit ((int)*cp) && (value = isdigit ((int)*cp) ? *cp - '0' : (islower ((int)*cp)
								 ?
								 toupper ((int)*cp)
								 : *cp) -
			    'A' + 10) < base)
    {
      result = result * base + value;
      cp++;
    }
  if (endp)
    *endp = (char *) cp;
  return result;
}

long
simple_strtol (const char *cp, char **endp, unsigned int base)
{
  if (*cp == '-')
    return -simple_strtoul (cp + 1, endp, base);
  return simple_strtoul (cp, endp, base);
}

/* we use this so that we can do without the ctype library */
#define is_digit(c)	((c) >= '0' && (c) <= '9')

static int
skip_atoi (const char **s)
{
  int i = 0;

  while (is_digit (**s))
    i = i * 10 + *((*s)++) - '0';
  return i;
}

int ten_power (int n) {
  int res = 1;
  while (n-- > 0) {
	res *= 10;
  }
  return res;
}

int ten_log (int n) {
  int res = 1;
  while (!(n < 10)) {
	n /= 10;
	res++;
  }
  return res;
}

#define ZEROPAD	1		/* pad with zero */
#define SIGN	2		/* unsigned/signed long */
#define PLUS	4		/* show plus */
#define SPACE	8		/* space if plus */
#define LEFT	16		/* left justified */
#define LARGE	64		/* use 'ABCDEF' instead of 'abcdef' */
#define FLOAT	128		/* floating point number */

#define do_div(n,base) ({ \
	int __res; \
	__res = ((unsigned long) n) % base; \
	n = ((unsigned long) n) / base; \
	__res; \
})

static void
number (long num, unsigned int base, int size, int precision,
	int type)
{
  char c, sign;
  static char tmp[66];

  const char *digits = "0123456789abcdefghijklmnopqrstuvwxyz";
  int i;

  if (type & LARGE)
    digits = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
  if (type & LEFT)
    type &= ~ZEROPAD;
  if (base < 2 || base > 36)
    return;
  c = (type & ZEROPAD) ? '0' : ' ';
  sign = 0;
  if (type & SIGN)
    {
      if (num < 0)
	{
	  sign = '-';
	  num = -num;
	  size--;
	}
      else if (type & PLUS)
	{
	  sign = '+';
	  size--;
	}
      else if (type & SPACE)
	{
	  sign = ' ';
	  size--;
	}
    }
  i = 0;
  if (num == 0)
    tmp[i++] = '0';
  else
    while (num != 0)
      tmp[i++] = digits[do_div (num, base)];
  if (i > precision)
    precision = i;
  size -= precision;
  if (!(type & (ZEROPAD + LEFT)))
    while (size-- > 0)
      DEBUG_SEND_HOOK(' ');
  if (sign)
    DEBUG_SEND_HOOK(sign);
  if (!(type & LEFT))
    while (size-- > 0)
      DEBUG_SEND_HOOK(c);
  while (i < precision--)
    DEBUG_SEND_HOOK('0');
  while (i-- > 0) {
    DEBUG_SEND_HOOK(tmp[i]);
  }
  while (size-- > 0)
    DEBUG_SEND_HOOK(' ');
}

static void
tiny_vsprintf (const char *fmt, va_list args)
{
  int len;
  unsigned long num;
  int i, base;
  const char *s;

  double initial_float;
  signed long pre_dec;
  long post_dec;

  int flags;			/* flags to number() */

  int field_width;		/* width of output field */
  int precision;		/* min. # of digits for integers; max
				   number of chars for from string */
  int qualifier;		/* 'h', 'l', or 'q' for integer fields */

  for (; *fmt; ++fmt)
    {
      if (*fmt != '%')
	{
	  if(*fmt=='\n')
	    DEBUG_SEND_HOOK('\r');
	  DEBUG_SEND_HOOK(*fmt);
	  continue;
	}

      /* process flags */
      flags = 0;
    repeat:
      ++fmt;			/* this also skips first '%' */
      switch (*fmt)
	{
	case '-':
	  flags |= LEFT;
	  goto repeat;
	case '+':
	  flags |= PLUS;
	  goto repeat;
	case ' ':
	  flags |= SPACE;
	  goto repeat;
	case '0':
	  flags |= ZEROPAD;
	  goto repeat;
	}

      /* get field width */
      field_width = -1;
      if (is_digit (*fmt))
	field_width = skip_atoi (&fmt);
      else if (*fmt == '*')
	{
	  ++fmt;
	  /* it's the next argument */
	  field_width = va_arg (args, int);
	  if (field_width < 0)
	    {
	      field_width = -field_width;
	      flags |= LEFT;
	    }
	}

      /* get the precision */
      precision = -1;
      if (*fmt == '.')
	{
	  ++fmt;
	  if (is_digit (*fmt))
	    precision = skip_atoi (&fmt);
	  else if (*fmt == '*')
	    {
	      ++fmt;
	      /* it's the next argument */
	      precision = va_arg (args, int);
	    }
	  if (precision < 0)
	    precision = 0;
	}

      /* get the conversion qualifier */
      qualifier = -1;
      if (*fmt == 'h' || *fmt == 'l' || *fmt == 'L' ||
	  *fmt == 'Z' || *fmt == 'z' || *fmt == 't' || *fmt == 'q')
	{
	  qualifier = *fmt;
	  if (qualifier == 'l' && *(fmt + 1) == 'l')
	    {
	      qualifier = 'q';
	      ++fmt;
	    }
	  ++fmt;
	}

      /* default base */
      base = 10;

      switch (*fmt)
	{
	case 'c':
	  if (!(flags & LEFT))
	    while (--field_width > 0)
	      DEBUG_SEND_HOOK(' ');
	  DEBUG_SEND_HOOK((unsigned char) va_arg (args, int));
	  while (--field_width > 0)
	    DEBUG_SEND_HOOK(' ');
	  continue;

	case 's':
	  s = va_arg (args, char *);
	  if (!s)
	    s = "<NULL>";

	  len = strnlen (s, precision);

	  if (!(flags & LEFT))
	    while (len < field_width--)
	      DEBUG_SEND_HOOK(' ');
	  for (i = 0; i < len; ++i)
	    DEBUG_SEND_HOOK(*s++);
	  while (len < field_width--)
	    DEBUG_SEND_HOOK(' ');
	  continue;

	case '%':
	  DEBUG_SEND_HOOK('%');
	  continue;

	  /* integer number formats - set up the flags and "break" */
	case 'o':
	  base = 8;
	  break;

	case 'X':
	  flags |= LARGE;
	case 'x':
	  base = 16;
	  break;

	case 'd':
	case 'i':
	  flags |= SIGN;
	case 'u':
	  break;

	case 'f':
	  flags |= FLOAT;
	  break;

	default:
	  DEBUG_SEND_HOOK('%');
	  if (*fmt) {
	    DEBUG_SEND_HOOK(*fmt);
	  } else {
	    --fmt;
	  }
	  continue;
	}

      if (qualifier == 'l')
	{
	  num = va_arg (args, unsigned long);
	}
      else if (qualifier == 'Z' || qualifier == 'z')
	{
	  num = va_arg (args, int); //unsigned short); //size_t);
	}
      else if (qualifier == 'h')
	{
	  num = (unsigned short) va_arg (args, int);
	  if (flags & SIGN)
	    num = (short) num;
	}
      else if (flags & SIGN)
	num = va_arg (args, int);
	  else if (flags & FLOAT)
	{
	  /* Default precision if none specified */
	  precision = (precision <= 0) ? 6 : precision;
	  /* Get value */
	  initial_float = va_arg (args, double);
	  /* Get integer preceding decimal point */
	  pre_dec = (long) initial_float;
	  /* Get integer following decimal point */
	  post_dec = (long)((initial_float - pre_dec) * ten_power(precision));
	  /* Check for and remove trailing 0's in post_dec */
	  for (i = 0; i < precision; i++) {

		if ((post_dec/ten_power(i))%10 != 0) {
			break;
		}
	  }
	  post_dec /= ten_power(i);
	  /* Send pre_dec to number(), width = specified_width - digits in post_dec - 1 for '.' */
	  number(pre_dec, 10, field_width-ten_log(post_dec) - 1, 
			-1, (pre_dec < 0) ? flags|SIGN: flags);
	  DEBUG_SEND_HOOK('.');
	  /* Load post_dec values to be sent to number() */
	  num = (post_dec < 0) ? post_dec * -1: post_dec;
	  /* Post decimal number must be zero padded */
	  flags |= ZEROPAD;
	  field_width = precision - i;
	  precision = 0;
	}
      else
	num = va_arg (args, unsigned int);
      number (num, base, field_width, precision, flags);
    }
}

void
debug_printf (const char *fmt, ...)
{
	va_list args;

	va_start (args, fmt);
	tiny_vsprintf (fmt, args);
	DEBUG_FLUSH_HOOK();
	va_end (args);
	DEBUG_FLUSH_DELAY();
}

void hex_dump (const unsigned char *buf, unsigned int addr, unsigned int len)
{
        unsigned int start, i, j;
        char c;

        start = addr & ~0xf;

        for (j=0; j<len; j+=16) {
                debug_printf("%08x:", start+j);

                for (i=0; i<16; i++) {
                        if (start+i+j >= addr && start+i+j < addr+len)
                                debug_printf(" %02x", buf[start+i+j]);
                        else
                                debug_printf("   ");
                }
                debug_printf("  |");
                for (i=0; i<16; i++) {
                        if (start+i+j >= addr && start+i+j < addr+len) {
                                c = buf[start+i+j];
                                if (c >= ' ' && c < 127)
                                        debug_printf("%c", c);
                                else
                                        debug_printf(".");
                        } else
                                debug_printf(" ");
                }
                debug_printf("|\n\r");
        }
}
