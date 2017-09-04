/*
 ## Cypress FX3 Device Firmware Source (cyu3utils.c)
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

#include <cyu3types.h>
#include <cyu3os.h>
#include <cyu3error.h>
#include <cyu3utils.h>

void
CyU3PMemCopy32 (
        uint32_t *dest, 
        uint32_t *src,
        uint32_t count)
{
    if (dest > src)
    {
        /* Destination buffer is above source buffer. We need to copy from end back to the start. */
        dest += count;
        src  += count;

        while (count >= 8)
        {
            dest  -= 8;
            src   -= 8;
            count -= 8;

            /* Loop unrolling for faster operation */
            dest[0] = src[0];
            dest[1] = src[1];
            dest[2] = src[2];
            dest[3] = src[3];
            dest[4] = src[4];
            dest[5] = src[5];
            dest[6] = src[6];
            dest[7] = src[7];
        }

        while (count > 0)
        {
            dest--;
            src--;
            count--;

            *dest = *src;
        }
    }
    else
    {
        /* Destination buffer is below source buffer. It is safe to copy from start to end. */
        while (count >= 8)
        {
            /* Loop unrolling for faster operation */
            dest[0] = src[0];
            dest[1] = src[1];
            dest[2] = src[2];
            dest[3] = src[3];
            dest[4] = src[4];
            dest[5] = src[5];
            dest[6] = src[6];
            dest[7] = src[7];

            dest  += 8;
            src   += 8;
            count -= 8;
        }

        while (count > 0)
        {
            *dest = *src;

            dest++;
            src++;
            count--;
        }
    }
}

/* Summary
 * This function is used to compute the checksum.
 */
CyU3PReturnStatus_t
CyU3PComputeChecksum (
        uint32_t *buffer,
        uint32_t  length,
        uint32_t *chkSum)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t count;

    /* Validate the parameters */
    if ((buffer == NULL) || (length == 0) || (chkSum == NULL))
    {
        status = CY_U3P_ERROR_BAD_ARGUMENT;
    }
    else
    {
        *chkSum = 0;
        for (count = 0; count < (length / 4); count++)
        {
            *chkSum += buffer[count];
        }
    }
    return status;
}

CyU3PReturnStatus_t
CyU3PReadDeviceRegisters (
        uvint32_t *regAddr,
        uint8_t    numRegs,
        uint32_t  *dataBuf)
{
    if ((dataBuf == 0) || (numRegs == 0) || (((uint32_t)regAddr & 0xF0000000) < 0xE0000000))
        return CY_U3P_ERROR_BAD_ARGUMENT;

    while (numRegs)
    {
        *dataBuf++ = *regAddr++;
        numRegs--;
    }

    return CY_U3P_SUCCESS;
}

CyU3PReturnStatus_t
CyU3PWriteDeviceRegisters (
        uvint32_t *regAddr,
        uint8_t    numRegs,
        uint32_t  *dataBuf)
{
    if ((dataBuf == 0) || (numRegs == 0) || (((uint32_t)regAddr & 0xF0000000) < 0xE0000000))
        return CY_U3P_ERROR_BAD_ARGUMENT;

    while (numRegs)
    {
        *regAddr++ = *dataBuf++;
        numRegs--;
    }

    return CY_U3P_SUCCESS;
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

void
__aeabi_memmove (
        void       *dest,
        const void *src,
        size_t      n)
{
    __aeabi_memcpy (dest, src, n);
}

void
__aeabi_memmove4 (
        void       *dest,
        const void *src,
        size_t      n)
{
    __aeabi_memcpy (dest, src, n);
}

#endif

/* [] */

