/*
; Cypress FX3 Firmware Entry code (cyu3vectors_gcc.S)
; ## =====================================================
; ##
; ##  Copyright Cypress Semiconductor Corporation, 2010-2014,
; ##  All Rights Reserved
; ##  UNPUBLISHED, LICENSED SOFTWARE.
; ##
; ##  CONFIDENTIAL AND PROPRIETARY INFORMATION
; ##  WHICH IS THE PROPERTY OF CYPRESS.
; ##
; ##  Use of this file is governed
; ##  by the license agreement included in the file
; ##
; ##     <install>/license/license.txt
; ##
; ##  where <install> is the Cypress software
; ##  installation root directory path.
; ##
; ## =======================================================
 */

#define CY_U3P_VIC_ADDRESS_OFFSET       0xFFFFFEE0

.extern CyU3PFirmwareEntry

/* The exception vectors imported below are to be defined by the application firmware. */
.extern CyU3PUndefinedHandler
.extern CyU3PPrefetchHandler
.extern CyU3PAbortHandler

.text
.code 32

.global __cyu3p_exception_vectors
__cyu3p_exception_vectors:
    ldr pc, =CyU3PFirmwareEntry
    ldr pc, =CyU3PUndefinedHandler
    ldr pc, =__CyU3PSWIHandler
    ldr pc, =CyU3PPrefetchHandler
    ldr pc, =CyU3PAbortHandler
    ldr pc, =__CyU3PReservedHandler
    ldr pc,  [PC, #CY_U3P_VIC_ADDRESS_OFFSET]
    ldr pc, =__CyU3PFiqHandler

/* The following exceptions are not expected to occur on this setup.
   The handlers are left in place for completion.
 */
.global __CyU3PSWIHandler
__CyU3PSWIHandler:
    b   __CyU3PSWIHandler

.global __CyU3PReservedHandler
__CyU3PReservedHandler:
    b   __CyU3PReservedHandler

.global __CyU3PFiqHandler
__CyU3PFiqHandler:
    b   __CyU3PFiqHandler

.end

