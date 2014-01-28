// -*- Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * vprintf for SITL
 *
 *  This firmware is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
*/

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL

#include <AP_Progmem.h>
#include <stdarg.h>
#include <string.h>
#include "print_vprintf.h"
#include <stdio.h>
#include <stdlib.h>

/*
  implement vprintf with support for %S meaning a progmem string
 */
void print_vprintf(AP_HAL::Print *s, unsigned char in_progmem, const char *fmt, va_list ap)
{
        char *str = NULL;
        int i;
        char *fmt2 = strdup(fmt);
        for (i=0; fmt2[i]; i++) {
                // cope with %S
                if (fmt2[i] == '%' && fmt2[i+1] == 'S') {
                        fmt2[i+1] = 's';
                }
        }
        (void)vasprintf(&str, fmt2, ap);
        for (i=0; str[i]; i++) {
                s->write(str[i]);
        }
        free(str);
        free(fmt2);
}


/*
  implement vsnprintf with support for %S meaning a progmem string
 */
int print_vsnprintf(char *s, size_t n, const char *fmt, va_list ap)
{
    int i, ret;
    char *fmt2 = (char *)fmt;
    if (strstr(fmt2, "%S") != NULL) {
        fmt2 = strdup(fmt);
        for (i=0; fmt2[i]; i++) {
            // cope with %S
            if (fmt2[i] == '%' && fmt2[i+1] == 'S') {
                fmt2[i+1] = 's';
            }
        }
    }
    ret = vsnprintf(s, n, fmt2, ap);
    if (fmt2 != fmt) {
        free(fmt2);
    }
    return ret;
}
#endif
