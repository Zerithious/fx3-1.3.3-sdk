/*
 ## Cypress FX3 Boot Firmware Header (cyfx3utils.h)
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

#ifndef __CYFX3UTILS_H__
#define __CYFX3UTILS_H__

#include <cyu3types.h>
#include <cyfx3error.h>
#include <cyfx3_api.h>
#include <cyu3externcstart.h>

/** \file cyfx3utils.h
    \brief This file provides some generic utility functions for the use of the
    FX3 boot library.
 */

/** \brief Delay subroutine.

    **Description**\n
    This function provides delays in multiple of microseconds. The delay is provided
    by a busy execution loop, which means that the CPU cannot perform any other
    operations while this code is being executed.
*/
extern void
CyFx3BootBusyWait (
        uint16_t usWait         /**< Delay duration in micro-seconds. */
        );

/* The busy wait function is provided by the FX3 mini library. */
#define CyFx3BootBusyWait(usWait)       CyFx3BusyWait(usWait)

/** \brief Copy one memory block to another.

    **Description**\n
    This function is a memcpy equivalent that copies data from one buffer to
    another. The copy is done byte-by-byte; and allows the two data buffers to
    overlap. No validity checks on the source and destination buffers are
    performed.

    **Return Value**\n
    * None
 */
extern void
CyFx3BootMemCopy (
        uint8_t *dest,                  /**< Pointer to destination buffer. */
        uint8_t *src,                   /**< Pointer to source buffer. */
        uint32_t count                  /**< Size of data to be copied, in bytes. */
        );

/** \brief Initialize a memory block with fixed data.

    **Description**\n
    This function is a memset equivalent that initializes all bytes in a memory block
    to the same value. The memset is done by updating each byte one at a time.

    **Return Value**\n
    * None
 */
extern void
CyFx3BootMemSet (
        uint8_t *buf,                   /**< Pointer to memory buffer. */
        uint8_t  value,                 /**< Value to be set at each byte. */
        uint32_t count                  /**< Size of the memory buffer, in bytes. */
        );

/** \brief Print a formatted string to a buffer.

    **Description**\n
    This function is a reduced version of the snprintf C library function, and prints
    a formatted string into a buffer based on the input arguments.

    The function only supports a subset of the output conversion specifications
    supported by the snprintf function. The 'c', 'd', 'u', 'x' and 's' conversion
    specifications are supported. There is no support for float or double formats,
    or for flags, precision or type modifiers.

    **Return Value**\n
    * CY_FX3_BOOT_SUCCESS if the formatted output is stored correctly.
    * CY_FX3_BOOT_ERROR_BAD_ARGUMENT if arguments are NULL, or if the target buffer is too short.
 */
CyFx3BootErrorCode_t
CyFx3BootSNPrintf (
        uint8_t  *buffer,               /**< Buffer into which the formatted string is to be printed. */
        uint16_t  maxLength,            /**< Length of the output buffer. */
        char     *fmt,                  /**< Format string. */
        ...                             /**< Variable list of parameters to be printed. */
        );

#ifndef CY_USE_ARMCC

/**< \brief Insert a single NOP instruction.

     **Description**\n
     This function inserts a single NOP instruction. This is a replacement for the __nop intrinsic
     that is provided by the ARM RVCT tool-chain. This is made an inline assembly macro to avoid
     un-necessary delays due to function calls.
 */
#define __nop()                                                 \
{                                                               \
    __asm__ __volatile__                                        \
    (                                                           \
        "mov r0, r0\n\t"                                        \
    );                                                          \
}

#endif

#include <cyu3externcend.h>

#endif /* __CYFX3UTILS_H__ */

