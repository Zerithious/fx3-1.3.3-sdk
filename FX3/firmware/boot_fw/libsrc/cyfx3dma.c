/*
 ## Cypress FX3 Boot Firmware Source (cyfx3dma.c)
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

#include <cyfx3dma.h>
#include <cyfx3device.h>
#include <cyfx3utils.h>
#include <cyfx3_api.h>
#include <cyfx3bootloader.h>
#include <pib_regs.h>
#include <lpp_regs.h>
#include <vic_regs.h>
#include <uib_regs.h>
#include <sib_regs.h>
#include <uibin_regs.h>
#include <sock_regs.h>

/*
 * @@DMA
 * Summary
 * This file provides functions to make use of the DMA capabilities of the FX3 device. Only low
 * level functions are provided, and high level constructs like DMA channels have to be setup in
 * user code.
 */

static CyFx3BootDmaCallback_t glDmaCallback   = 0;      /* DMA callback registered by user. */
static uint32_t               glDmaIntrEnable = 0;      /* DMA interrupt enable mask. */

/* ISR for DMA interrupts from the USB block. */
#ifdef CY_USE_ARMCC
static void
usb_dma_isr (
        void) __irq
#else
static void __attribute__((interrupt("IRQ"))) usb_dma_isr (
        void)
#endif
{
    uint32_t intStat;
    uint32_t sock;
    uint32_t value;

    /* Check whether any interrupts are active on the USB egress sockets and raise callbacks for the same. */
    intStat = CY_U3P_UIB_SCK_INTR0;
    if (intStat != 0)
    {
        for (sock = 0; sock < CY_FX3_DMA_USB_IN_SOCKCNT; sock++)
        {
            if ((intStat & (1 << sock)) != 0)
            {
                value = CyFx3BootDmaGetSockInterrupts ((CyFx3BootDmaSockId_t)(CY_DMA_UIB_SOCKET_CONS_0 + sock));
                if (glDmaCallback != 0)
                {
                    glDmaCallback ((CY_DMA_UIB_SOCKET_CONS_0 + sock), value);
                }
                else
                {
                    CyFx3BootDmaClearSockInterrupts ((CyFx3BootDmaSockId_t)(CY_DMA_UIB_SOCKET_CONS_0 + sock),
                            value);
                }
            }
        }
    }

    /* Check whether any interrupts are active on the USB ingress sockets and raise callbacks for the same. */
    intStat = CY_U3P_UIBIN_SCK_INTR0;
    if (intStat != 0)
    {
        for (sock = 0; sock < CY_FX3_DMA_USB_OUT_SOCKCNT; sock++)
        {
            if ((intStat & (1 << sock)) != 0)
            {
                value = CyFx3BootDmaGetSockInterrupts ((CyFx3BootDmaSockId_t)(CY_DMA_UIB_SOCKET_PROD_0 + sock));
                if (glDmaCallback != 0)
                {
                    glDmaCallback ((CY_DMA_UIB_SOCKET_PROD_0 + sock), value);
                }
                else
                {
                    CyFx3BootDmaClearSockInterrupts ((CyFx3BootDmaSockId_t)(CY_DMA_UIB_SOCKET_PROD_0 + sock),
                            value);
                }
            }
        }
    }

    VIC->address = 0;
}

/* ISR for DMA interrupts from the PIB block. */
#ifdef CY_USE_ARMCC
static void
pib_dma_isr (
        void) __irq
#else
static void __attribute__((interrupt("IRQ"))) pib_dma_isr (
        void)
#endif
{
    uint32_t intStat;
    uint32_t sock;
    uint32_t value;

    /* Check whether any interrupts are active on the USB egress sockets and raise callbacks for the same. */
    intStat = CY_U3P_PIB_SCK_INTR0;
    if (intStat != 0)
    {
        for (sock = 0; sock < CY_FX3_DMA_PIB_SOCKCNT; sock++)
        {
            if ((intStat & (1 << sock)) != 0)
            {
                value = CyFx3BootDmaGetSockInterrupts ((CyFx3BootDmaSockId_t)(CY_DMA_PIB_SOCKET_0 + sock));
                if (glDmaCallback != 0)
                {
                    glDmaCallback ((CY_DMA_PIB_SOCKET_0 + sock), value);
                }
                else
                {
                    CyFx3BootDmaClearSockInterrupts ((CyFx3BootDmaSockId_t)(CY_DMA_PIB_SOCKET_0 + sock),
                            value);
                }
            }
        }
    }

    VIC->address = 0;
}

/* ISR for DMA interrupts from the serial peripheral block. */
#ifdef CY_USE_ARMCC
static void
lpp_dma_isr (
        void) __irq
#else
static void __attribute__((interrupt("IRQ"))) lpp_dma_isr (
        void)
#endif
{
    uint32_t intStat;
    uint32_t sock;
    uint32_t value;

    /* Check whether any interrupts are active on the USB egress sockets and raise callbacks for the same. */
    intStat = CY_U3P_LPP_SCK_INTR0;
    if (intStat != 0)
    {
        for (sock = 0; sock < CY_FX3_DMA_LPP_SOCKCNT; sock++)
        {
            if ((intStat & (1 << sock)) != 0)
            {
                value = CyFx3BootDmaGetSockInterrupts ((CyFx3BootDmaSockId_t)(CY_DMA_LPP_SOCKET_I2S_LEFT + sock));
                if (glDmaCallback != 0)
                {
                    glDmaCallback ((CY_DMA_LPP_SOCKET_I2S_LEFT + sock), value);
                }
                else
                {
                    CyFx3BootDmaClearSockInterrupts ((CyFx3BootDmaSockId_t)(CY_DMA_LPP_SOCKET_I2S_LEFT + sock),
                            value);
                }
            }
        }
    }

    VIC->address = 0;
}

void
CyFx3BootDmaRegisterCallback (
        CyFx3BootDmaCallback_t cbFunc,
        CyBool_t               usbIntrEn,
        CyBool_t               pibIntrEn,
        CyBool_t               serialIntrEn)
{
    if (cbFunc != 0)
    {
        /* Valid callback has been provided. Register it and enable interrupts requested.
         * Interrupts for a block should only be enabled if the block is powered on.
         */
        glDmaCallback = cbFunc;
        if (usbIntrEn)
        {
            if (CyFx3UsbIsOn ())
            {
                VIC->vec_address[FX3_INTR_USB_DMA] = (uint32_t)usb_dma_isr;
                VIC->int_enable                    = (1 << FX3_INTR_USB_DMA);
            }
            glDmaIntrEnable                   |= (1 << FX3_INTR_USB_DMA);
        }

        if (pibIntrEn)
        {
            if (CyFx3PibIsOn ())
            {
                VIC->vec_address[FX3_INTR_PIB_DMA] = (uint32_t)pib_dma_isr;
                VIC->int_enable                    = (1 << FX3_INTR_PIB_DMA);
            }
            glDmaIntrEnable                       |= (1 << FX3_INTR_PIB_DMA);
        }

        if (serialIntrEn)
        {
            if (CyFx3LppIsOn ())
            {
                VIC->vec_address[FX3_INTR_LPP_DMA] = (uint32_t)lpp_dma_isr;
                VIC->int_enable                    = (1 << FX3_INTR_LPP_DMA);
            }
            glDmaIntrEnable                       |= (1 << FX3_INTR_LPP_DMA);
        }
    }
    else
    {
        /* Callback has been unregistered. Disable any active DMA interrupts. */
        VIC->int_clear  = (1 << FX3_INTR_USB_DMA) | (1 << FX3_INTR_PIB_DMA) | (1 << FX3_INTR_LPP_DMA);
        glDmaIntrEnable = 0;
        glDmaCallback   = 0;
    }
}

CyFx3BootErrorCode_t
CyFx3BootDmaGetDscrConfig (
        uint16_t                  dscrIndex,
        CyFx3BootDmaDescriptor_t *dscr_p)
{
    CyFx3BootDmaDescriptor_t *curDscr_p;

    if ((dscrIndex < CY_FX3_DMA_MIN_DSCR_INDEX) || (dscrIndex > CY_FX3_DMA_MAX_DSCR_INDEX) || (dscr_p == 0))
        return CY_FX3_BOOT_ERROR_BAD_ARGUMENT;

    curDscr_p = CY_FX3_DMA_GETDSCR_BY_INDEX (dscrIndex);
    *dscr_p   = *curDscr_p;

    return CY_FX3_BOOT_SUCCESS;
}

CyFx3BootErrorCode_t
CyFx3BootDmaSetDscrConfig (
        uint16_t                  dscrIndex,
        CyFx3BootDmaDescriptor_t *dscr_p)
{
    CyFx3BootDmaDescriptor_t *curDscr_p;

    if ((dscrIndex < CY_FX3_DMA_MIN_DSCR_INDEX) || (dscrIndex > CY_FX3_DMA_MAX_DSCR_INDEX) || (dscr_p == 0))
        return CY_FX3_BOOT_ERROR_BAD_ARGUMENT;

    curDscr_p  = CY_FX3_DMA_GETDSCR_BY_INDEX (dscrIndex);
    *curDscr_p = *dscr_p;

    return CY_FX3_BOOT_SUCCESS;
}

/* Get the socket register address for a given socket id. */
#ifdef CY_USE_ARMCC
static __forceinline CyFx3BootDmaSockRegs_t *
#else
static CyFx3BootDmaSockRegs_t *
#endif
CyFx3BootDmaGetSocketPtr (
        CyFx3BootDmaSockId_t sockId)
{
    /* LPP socket. */
    if (sockId <= CY_DMA_LPP_SOCKET_SPI_PROD)
        return ((CyFx3BootDmaSockRegs_t *)(CY_U3P_LPP_SCK_DSCR_ADDRESS (sockId - CY_DMA_LPP_SOCKET_I2S_LEFT)));

    /* PIB socket. */
    if ((sockId >= CY_DMA_PIB_SOCKET_0) && (sockId <= CY_DMA_PIB_SOCKET_31))
        return ((CyFx3BootDmaSockRegs_t *)(CY_U3P_PIB_SCK_DSCR_ADDRESS (sockId - CY_DMA_PIB_SOCKET_0)));

    /* SIB socket. Not supported in the rest of the library as of now. */
    if ((sockId >= CY_DMA_SIB_SOCKET_0) && (sockId <= CY_DMA_SIB_SOCKET_5))
        return ((CyFx3BootDmaSockRegs_t *)(CY_U3P_SIB_SCK_DSCR_ADDRESS (sockId - CY_DMA_SIB_SOCKET_0)));

    /* USB Egress Socket. */
    if ((sockId >= CY_DMA_UIB_SOCKET_CONS_0) && (sockId <= CY_DMA_UIB_SOCKET_CONS_15))
        return ((CyFx3BootDmaSockRegs_t *)(CY_U3P_UIB_SCK_DSCR_ADDRESS (sockId - CY_DMA_UIB_SOCKET_CONS_0)));

    /* USB Ingress Socket. */
    if ((sockId >= CY_DMA_UIB_SOCKET_PROD_0) && (sockId <= CY_DMA_UIB_SOCKET_PROD_15))
        return ((CyFx3BootDmaSockRegs_t *)(CY_U3P_UIBIN_SCK_DSCR_ADDRESS (sockId - CY_DMA_UIB_SOCKET_PROD_0)));

    /* Invalid socket. */
    return (0);
}

CyFx3BootErrorCode_t
CyFx3BootDmaGetSocketConfig (
        CyFx3BootDmaSockId_t  sockId,
        CyFx3BootDmaSocket_t *sock_p)
{
    CyFx3BootDmaSockRegs_t *sockReg_p = CyFx3BootDmaGetSocketPtr (sockId);

    /* Invalid socket or config structure. */
    if ((sockReg_p == 0) || (sock_p == 0))
        return CY_FX3_BOOT_ERROR_BAD_ARGUMENT;

    /* Copy relevant fields into the return structure. */
    sock_p->dscrChain = sockReg_p->dscrChain;
    sock_p->xferSize  = sockReg_p->xferSize;
    sock_p->xferCount = sockReg_p->xferCount;
    sock_p->status    = sockReg_p->status;
    sock_p->intr      = sockReg_p->intr;
    sock_p->intrMask  = sockReg_p->intrMask;

    return CY_FX3_BOOT_SUCCESS;
}

CyFx3BootErrorCode_t
CyFx3BootDmaSetSocketConfig (
        CyFx3BootDmaSockId_t  sockId,
        CyFx3BootDmaSocket_t *sock_p)
{
    CyFx3BootDmaSockRegs_t *sockReg_p = CyFx3BootDmaGetSocketPtr (sockId);

    /* Invalid socket or config structure. */
    if ((sockReg_p == 0) || (sock_p == 0))
        return CY_FX3_BOOT_ERROR_BAD_ARGUMENT;

    /* Copy relevant fields into the return structure. */
    sockReg_p->dscrChain = sock_p->dscrChain;
    sockReg_p->xferSize  = sock_p->xferSize;
    sockReg_p->xferCount = sock_p->xferCount;
    sockReg_p->intr      = sock_p->intr;
    sockReg_p->intrMask  = sock_p->intrMask;
    sockReg_p->status    = sock_p->status;

    return CY_FX3_BOOT_SUCCESS;
}

CyFx3BootErrorCode_t
CyFx3BootDmaWrapSocket (
        CyFx3BootDmaSockId_t sockId)
{
    CyFx3BootDmaSockRegs_t *sockReg_p = CyFx3BootDmaGetSocketPtr (sockId);

    /* Invalid socket. */
    if (sockReg_p == 0)
        return CY_FX3_BOOT_ERROR_BAD_ARGUMENT;

    /* Set the wrap-up bit if the socket is enabled. */
    if ((sockReg_p->status & CY_U3P_ENABLED) != 0)
    {
        sockReg_p->status = sockReg_p->status | CY_U3P_WRAPUP;
    }

    return CY_FX3_BOOT_SUCCESS;
}

CyFx3BootErrorCode_t
CyFx3BootDmaDisableSocket (
        CyFx3BootDmaSockId_t sockId)
{
    CyFx3BootDmaSockRegs_t *sockReg_p = CyFx3BootDmaGetSocketPtr (sockId);

    /* Invalid socket. */
    if (sockReg_p == 0)
        return CY_FX3_BOOT_ERROR_BAD_ARGUMENT;

    /* Do nothing if the socket is already disabled. */
    if ((sockReg_p->status & CY_U3P_ENABLED) != 0)
    {
        /* Clear the socket enable bit and wait until the socket is no longer enabled. */
        sockReg_p->status = (sockReg_p->status & ~(CY_U3P_GO_ENABLE | CY_U3P_WRAPUP));
        while ((sockReg_p->status & CY_U3P_ENABLED) != 0);
    }

    return CY_FX3_BOOT_SUCCESS;
}

CyFx3BootErrorCode_t
CyFx3BootDmaEnableSocket (
        CyFx3BootDmaSockId_t sockId)
{
    CyFx3BootDmaSockRegs_t *sockReg_p = CyFx3BootDmaGetSocketPtr (sockId);

    /* Invalid socket. */
    if (sockReg_p == 0)
        return CY_FX3_BOOT_ERROR_BAD_ARGUMENT;

    if ((sockReg_p->status & CY_U3P_ENABLED) != 0)
        return CY_FX3_BOOT_ERROR_ALREADY_STARTED;

    /* Set the enable bit and wait until the socket is enabled. */
    sockReg_p->status = sockReg_p->status | CY_U3P_GO_ENABLE;
    while ((sockReg_p->status & CY_U3P_ENABLED) == 0);

    return CY_FX3_BOOT_SUCCESS;
}

CyFx3BootErrorCode_t
CyFx3BootDmaSendSocketEvent (
        CyFx3BootDmaSockId_t sockId,
        uint16_t             dscrIndex,
        CyBool_t             isOccupied)
{
    CyFx3BootDmaSockRegs_t *sockReg_p = CyFx3BootDmaGetSocketPtr (sockId);
    uint32_t evtVal = (uint32_t)dscrIndex;

    /* Invalid socket. */
    if (sockReg_p == 0)
        return CY_FX3_BOOT_ERROR_BAD_ARGUMENT;

    if (isOccupied)
        evtVal |= CY_U3P_EVENT_EVENT_TYPE;

    sockReg_p->sckEvent = evtVal;
    return CY_FX3_BOOT_SUCCESS;
}

void
CyFx3BootDmaClearSockInterrupts (
        CyFx3BootDmaSockId_t sockId,
        uint32_t             intrVal)
{
    CyFx3BootDmaSockRegs_t *sockReg_p = CyFx3BootDmaGetSocketPtr (sockId);

    if (sockReg_p != 0)
        sockReg_p->intr = intrVal;
}

uint32_t
CyFx3BootDmaGetSockInterrupts (
        CyFx3BootDmaSockId_t sockId)
{
    uint32_t intStat = 0;
    CyFx3BootDmaSockRegs_t *sockReg_p = CyFx3BootDmaGetSocketPtr (sockId);

    if (sockReg_p != 0)
        intStat = sockReg_p->intr;
    return intStat;
}

/*[]*/

