/*
 ## Cypress FX3 Boot Firmware Source (cyfx3utils.c)
 ## ===========================
 ##
 ##  Copyright Cypress Semiconductor Corporation, 2010-2014,
 ##  All Rights Reserved
 ##  UNPUBLISHED, LICENSED SOFTWARE.
 ##
 ##  CONFIDENTIAL AND PROPRIETARY INFORMATION
 ##  WHICH IS THE PROPERTY OF CYPRESS.
 ##
 ##  Use of this file is governed
 ##  by the license agreement included in the file
 ##
 ##     <install>/license/license.txt
 ##
 ##  where <install> is the Cypress software
 ##  installation root directory path.
 ##
 ## ===========================
 */

#include "cyfx3utils.h"
#include "cyfx3error.h"
#include <stdarg.h>

void
CyFx3BootMemCopy (
        uint8_t  *dest,
        uint8_t  *src,
        uint32_t  count)
{
    uint32_t i;

    /* It is possible that the source and destination buffers overlap. Make sure that the copy is done in
       the direction which allows a safe copy.
     */
    if (dest > src)
    {
        dest += count - 1;
        src  += count - 1;
        for (i = 0; i < count; i++)
        {
            *dest-- = *src--;
        }
    }
    else
    {
        for (i = 0; i < count; i++)
        {
            *dest++ = *src++;
        }
    }
}

void
CyFx3BootMemSet (
        uint8_t *buf,
        uint8_t  value,
        uint32_t count)
{
    while (count--)
    {
        *buf++ = value;
    }
}

/*
 * Converts a number to a string depending on base (hex or int) e.g. 10 to "A" or "10"
 */
static uint8_t *
DebugIntToStr (
        uint8_t  *convertedString,
        uint32_t  num,
        uint8_t   base)
{
    uint8_t *str_p, i = 10;

    str_p    = convertedString;
    str_p[i] = '\0';

    do
    {
        str_p[--i] = "0123456789ABCDEF"[num%base];
        num /= base;
    } while (num != 0);

    return (&str_p[i]);
}

static uint16_t
DebugGetStrLen (
        uint8_t *string_p)
{
    uint16_t len = 0;

    if (string_p != 0)
    {
        while (*string_p++ != '\0')
            len++;
    }

    return len;
}

static uint16_t
DebugStrCopy (
        uint8_t  *dest,
        uint8_t  *src)
{
    uint16_t len = 0;

    if ((dest != 0) && (src != 0))
    {
        while ((*dest++ = *src++) != '\0')
            len++;
    }

    return len;
}

CyFx3BootErrorCode_t
DebugSNPrint (
        uint8_t  *debugMsg,
        uint16_t *length,
        char     *message,
        va_list   argp)
{
    uint8_t  *string_p;
    uint8_t  *argStr = 0;
    CyBool_t  copyReqd = CyFalse;
    uint16_t  i = 0, j, maxLength = *length;
    int32_t   intArg;
    uint32_t  uintArg;
    uint8_t   convertedString[11];

    if (debugMsg == 0)
        return CY_FX3_BOOT_ERROR_BAD_ARGUMENT;

    /* Parse the string and copy into the buffer for sending out. */
    for (string_p = (uint8_t *)message; (*string_p != '\0'); string_p++)
    {
        if (i >= (maxLength - 2))
            return CY_FX3_BOOT_ERROR_BAD_ARGUMENT;

        if (*string_p != '%')
        {
            debugMsg[i++] = *string_p;
            continue;
        }

        string_p++;
        switch (*string_p)
        {
        case '%' :
            {
                debugMsg[i++] = '%';
            }
            break;

        case 'c' : 
            {
                debugMsg[i++] = (uint8_t)va_arg (argp, int32_t);
            }
            break;

        case 'd' : 
            {
                intArg = va_arg (argp, int32_t);
                if (intArg < 0)
                {
                    debugMsg[i++] = '-';
                    intArg = -intArg;
                }

                argStr = DebugIntToStr (convertedString, intArg, 10);
                copyReqd = CyTrue;
            }
            break;

        case 's': 
            {
                argStr = va_arg (argp, uint8_t *); 
                copyReqd = CyTrue;
            }
            break;

        case 'u': 
            {
                uintArg = va_arg (argp, uint32_t); 
                argStr = DebugIntToStr (convertedString, uintArg, 10);
                copyReqd = CyTrue;
            }
            break;

        case 'X':
        case 'x': 
            {
                uintArg = va_arg (argp, uint32_t); 
                argStr = DebugIntToStr (convertedString, uintArg, 16);
                copyReqd = CyTrue;
            }
            break;

        default:
            return CY_FX3_BOOT_ERROR_BAD_ARGUMENT;
        }

        if (copyReqd)
        {
            j = DebugGetStrLen (argStr);
            if (i >= (maxLength - j - 1))
                return CY_FX3_BOOT_ERROR_BAD_ARGUMENT;
            DebugStrCopy ((debugMsg + i), argStr);
            i += j;
            copyReqd = CyFalse;
        }
    }

    /* NULL-terminate the string. There will always be space for this. */
    debugMsg[i] = '\0';
    *length     = i;

    return CY_FX3_BOOT_SUCCESS;
}

CyFx3BootErrorCode_t
CyFx3BootSNPrintf (
        uint8_t *buffer,
        uint16_t maxLength,
        char    *fmt,
        ...)
{
    CyFx3BootErrorCode_t stat;
    uint16_t len = maxLength;
    va_list  argp;
   
    va_start (argp, fmt);
    stat = DebugSNPrint (buffer, &len, fmt, argp);
    va_end (argp);

    return stat;
}

#ifdef CY_USE_ARMCC

/* Provide functions that are normally part of the C library for the ARM RVCT tool-chain. As we may be linking
   with GCC provided C libraries, we cannot count on these being provided by the library.

   Public prototypes are not required because we do not call these functions directly.
 */

#include <stddef.h>

void
__aeabi_memset (
        void   *dest,
        size_t  n,
        int     c)
{
    uint8_t *ptr = (uint8_t *)dest;
    while (n--)
    {
        *ptr++ = (uint8_t)c;
    }
}

void
__aeabi_memset4 (
        void   *dest,
        size_t  n,
        int     c)
{
    __aeabi_memset (dest, n, c);
}

void
__aeabi_memclr (
        void   *dest,
        size_t  n)
{
    __aeabi_memset (dest, n, 0);
}

void __aeabi_memclr4 (
        void   *dest,
        size_t  n)
{
    __aeabi_memset (dest, n, 0);
}

void
__aeabi_memcpy (
        void       *dest,
        const void *src,
        size_t      n)
{
    uint8_t *pd = (uint8_t *)dest;
    uint8_t *ps = (uint8_t *)src;

    if (dest < src)
    {
        while (n--)
        {
            *pd++ = *ps++;
        }
    }
    else
    {
        pd = (uint8_t *)dest + n - 1;
        ps = (uint8_t *)src + n - 1;
        while (n--)
        {
            *pd-- = *ps--;
        }
    }
}

void
__aeabi_memcpy4 (
        void       *dest,
        const void *src,
        size_t      n)
{
    __aeabi_memcpy (dest, src, n);
}

#endif

/* [] */
