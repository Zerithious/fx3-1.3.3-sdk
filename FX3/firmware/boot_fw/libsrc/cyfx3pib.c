/*
 ## Cypress FX3 Boot Firmware Source (cyfx3pib.c)
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

#include <cyfx3pib.h>
#include <cyfx3device.h>
#include <cyfx3utils.h>
#include <cyfx3dma.h>
#include <cyfx3_api.h>
#include <cyfx3bootloader.h>
#include <gctl_regs.h>
#include <pib_regs.h>
#include <pmmc_regs.h>
#include <lpp_regs.h>
#include <vic_regs.h>

/*
 * @@PIB
 * Summary
 * This file provides functions to enable the PMMC interface on the FX3 device.
 */

#define CYFX3_PIB_DMA_ID                (1)             /* DMA block ID for the PIB. */
#define CYFX3_DMA_MAX_SIZE              (0xFFF0)        /* Max size of single DMA buffer. */

#define CY_FX3_PMMC_PSN                 (0x01)          /* 32-bit product serial # */
#define CY_FX3_PMMC_PRV                 (0x01)          /* Product version # */
#define CY_FX3_PMMC_PNM_0               (0)             /* Lowest 8-bits of PNM */
#define CY_FX3_PMMC_PNM_1               (1)             /* 32 middle bits of PNM */
#define CY_FX3_PMMC_PNM_2               (2)             /* Upper 8 bits of PNM */
#define CY_FX3_PMMC_OID                 (3)             /* OEM ID. Must be changed */
#define CY_FX3_PMMC_CBX                 (0x01)          /* Device type: 1 = BGA */
#define CY_FX3_PMMC_MANUFACTURE_DATE    (0x92)          /* Manufacturing date and month */
#define CY_FX3_PMMC_MID                 (0x04)          /* Manufacturer ID. Must be changed */

/* CSD register */
#define CY_FX3_PMMC_CSD_LEN             (4)
static const uint32_t glPmmcCsdValue[CY_FX3_PMMC_CSD_LEN] =
{
    0x06E00C01,
    0xC0038000,
    0x415BF3FF,
    0x1001005A
};

/* Extended CSD register */
#define CY_FX3_PMMC_EXT_CSD_LEN         (6)
static const uint32_t glPmmcExtCsdValue[CY_FX3_PMMC_EXT_CSD_LEN] =
{
    0x40040000,
    0x03020200,
    0x00000404,
    0x32321414,
    0x00406464,
    0x00000000
};

/* Variables in file scope. */
static CyBool_t              glBootPibStarted = CyFalse;        /* Whether module has been started. */
static CyBool_t              glBootPibMmcMode = CyFalse;        /* Whether module is configured in MMC mode. */
static CyFx3BootPMMCIntrCb_t glBootPibMmcCallback = 0;          /* MMC event callback. */
static CyFx3BootGpifIntrCb_t glBootGpifCallback = 0;            /* Gpif event callback. */

static void
CyFx3BootPmmcInit (
        void)
{
    PMMC->intr          = 0xFFFFFFFF;
    PMMC->intr_mask     = 0;
    PMMC->sck_direction = 0xFFFF0000;
    PIB->intr_mask      = 0;

    /* Force the state */
    PMMC->config &= ~CY_U3P_PIB_CSR_CURRENT_STATE_MASK;

    /* Init the ID register */
    PMMC->cid0 =
        CY_U3P_PIB_PMMC_NOT_USED |                                  /* Always 1 */
        (CY_FX3_PMMC_MANUFACTURE_DATE << CY_U3P_PIB_PMMC_MDT_POS) | /* Manufacturing date */
        (CY_FX3_PMMC_PSN << CY_U3P_PIB_PMMC_PSN_L_POS);             /* LSB 16bit of PSN code */

    PMMC->cid1 =
        (CY_FX3_PMMC_PSN >> CY_U3P_PIB_PMMC_PSN_H_POS) |            /* Prod serial top 16 bit# */
        (CY_FX3_PMMC_PRV << CY_U3P_PIB_PMMC_PRV_POS) |              /* 8 bit Prod version # */
        (CY_FX3_PMMC_PNM_0 << CY_U3P_PIB_PMMC_PNM_L_POS);           /* 6 ASCII chrs Prod Name */

    PMMC->cid2 = CY_FX3_PMMC_PNM_1;                                 /* 8-40 of Product Name */

    PMMC->cid3 =
        (CY_FX3_PMMC_PNM_2 << CY_U3P_PIB_PMMC_PNM_H_POS) |          /* last 8 bits of Prod Name */
        (CY_FX3_PMMC_OID << CY_U3P_PIB_PMMC_OID_POS) |              /* 8-bit OID # */
        (CY_FX3_PMMC_CBX << CY_U3P_PIB_PMMC_CBX_POS) |              /* Device Package type */
        (CY_FX3_PMMC_MID << CY_U3P_PIB_PMMC_MID_POS) ;              /* Manufacturer ID */


    /* Init PMMC-CSD Card Specific Data register here */
    PMMC->csd0 = glPmmcCsdValue[0];
    PMMC->csd1 = glPmmcCsdValue[1];
    PMMC->csd2 = glPmmcCsdValue[2];
    PMMC->csd3 = glPmmcCsdValue[3];

    /* Setup extended CSD register */
    PMMC->ext_csd0 = glPmmcExtCsdValue[0];
    PMMC->ext_csd1 = glPmmcExtCsdValue[1];
    PMMC->ext_csd2 = glPmmcExtCsdValue[2];
    PMMC->ext_csd3 = glPmmcExtCsdValue[3];
    PMMC->ext_csd4 = glPmmcExtCsdValue[4];
    PMMC->ext_csd5 = glPmmcExtCsdValue[5];

    /* Init OCR register */
    PMMC->ocr = CY_U3P_PIB_PMMC_OCR_DEFAULT;

    /* Init CSR register */
    PMMC->csr = CY_U3P_PIB_PMMC_CSR_DEFAULT;

    /* Set a default PMMC block length */
    /* May not be necessary since Boot-ROM or host will set */
    PMMC->block_len = CY_U3P_PIB_PMMC_BLOCK_LEN_DEFAULT;
    PMMC->dir_sock  = 0x00000000;

    /* Set the proper INTR mask in PMMC module */
    PMMC->intr_mask = (CY_U3P_PIB_PMMC_GO_IDLE | CY_U3P_PIB_PMMC_CMD1 | CY_U3P_PIB_PMMC_CMD5_SLEEP |
            CY_U3P_PIB_PMMC_CMD5_AWAKE | CY_U3P_PIB_PMMC_CMD6 | CY_U3P_PIB_PMMC_CMD12 | CY_U3P_PIB_PMMC_CMD15 |
            CY_U3P_PIB_PMMC_NEW_CMD_RCVD | CY_U3P_PIB_PMMC_WR_SCK_NOT_RDY | CY_U3P_PIB_PMMC_RD_SCK_NOT_RDY);

    /* Set the proper INTR mask in PIB module */
    PIB->intr      = 0xFFFFFFFF;
    PIB->intr_mask = (CY_U3P_PIB_INTR_PMMC_INTR1);

    /* Set the OCR ready status to indicate that the firmware is ready for operation. */
    PMMC->ocr |= CY_U3P_PIB_PMMC_OCR_STATUS;
    CyFx3BootBusyWait (1);

    PIB->config = (CY_U3P_PIB_ENABLE | CY_U3P_PIB_PCFG | CY_U3P_PIB_PMMC_RESETN) |
        (PIB->config & (CY_U3P_PIB_MMIO_ENABLE | CY_U3P_PIB_DEVICE_ID_MASK));

    /* Set all PIB sockets in packet mode. */
    PIB->eop_eot = 0xFFFFFFFF;
}

#ifdef CY_USE_ARMCC
static void
pib_isr (
        void) __irq
#else
static void __attribute__((interrupt("IRQ"))) pib_isr (
        void)
#endif
{
    if ((PIB->intr & CY_U3P_PIB_INTR_GPIF_INTERRUPT) != 0)
    {
        if ((PIB->gpif_intr & CY_U3P_GPIF_INTR_GPIF_INTR) != 0)
        {
            /* If the user has registered a callback, notify him. */
            if (glBootGpifCallback != 0)
                glBootGpifCallback ((uint8_t)(CY_U3P_PIB_GPIF_STATUS >> CY_U3P_GPIF_STATUS_INTERRUPT_STATE_POS));
        }

        /* Clear the interrupt. */
        PIB->gpif_intr = PIB->gpif_intr;
    }
    else
    {
        /* Clear and disable all interrupts except GPIF. */
        PIB->intr      = PIB->intr & ~CY_U3P_PIB_INTR_GPIF_INTERRUPT;
        PIB->intr_mask = PIB->intr_mask & CY_U3P_PIB_INTR_GPIF_INTERRUPT;
    }

    VIC->address = 0;
}

static void
CyFx3BootGpifRegisterIsr (
        void)
{
    /* Enable the GPIF interrupt and set the interrupt vector. */
    VIC->vec_address[FX3_INTR_PIB_CORE] = (uint32_t)pib_isr;
    PIB->intr       = PIB->intr;
    PIB->intr_mask  = CY_U3P_PIB_INTR_GPIF_INTERRUPT;
    VIC->int_enable = (1 << FX3_INTR_PIB_CORE);
}

static void
CyFx3BootGpifUnregisterIsr (
        void)
{
    PIB->intr_mask = 0;
    VIC->int_clear = (1 << FX3_INTR_PIB_CORE);
}

static void
CyFx3BootGpifInit (
        void)
{
    CyFx3BootGpifRegisterIsr ();
}

CyFx3BootErrorCode_t
CyFx3BootPibInit (
        CyFx3BootPibClock_t *clkInfo_p,
        CyBool_t             isMmcMode)
{
    if (glBootPibStarted)
        return CY_FX3_BOOT_ERROR_ALREADY_STARTED;
    if (!CyFx3DevIsGpifSupported ())
        return CY_FX3_BOOT_ERROR_NOT_SUPPORTED;
    if (clkInfo_p == 0)
        return CY_FX3_BOOT_ERROR_BAD_ARGUMENT;

    /* Set the PIB clock frequency as desired and enable the clock. */
    GCTL->pib_core_clk = (((uint32_t)clkInfo_p->clkSrc << CY_U3P_GCTL_PIBCLK_SRC_POS) & CY_U3P_GCTL_PIBCLK_SRC_MASK) |
        ((((uint32_t)clkInfo_p->clkDiv - 1) << CY_U3P_GCTL_PIBCLK_DIV_POS) & CY_U3P_GCTL_PIBCLK_DIV_MASK);
    if (clkInfo_p->isHalfDiv)
    {
        GCTL->pib_core_clk |= CY_U3P_GCTL_PIBCLK_HALFDIV;
    }

    GCTL->pib_core_clk |= CY_U3P_GCTL_PIBCLK_CLK_EN;

    /* Power up the Pport and wait for reset to be completed. */
    CyFx3PibPowerOn ();

    /* Turn on the DLL if required. */
    if (clkInfo_p->isDllEnable)
    {
        CyFx3PibDllEnable ();
    }

    if (isMmcMode)
        CyFx3BootPmmcInit ();
    else
        CyFx3BootGpifInit ();

    glBootPibStarted = CyTrue;
    glBootPibMmcMode = isMmcMode;

    return CY_FX3_BOOT_SUCCESS;
}

CyFx3BootErrorCode_t
CyFx3BootPibDeinit (
        void)
{
    if (!glBootPibStarted)
        return CY_FX3_BOOT_ERROR_NOT_STARTED;

    /* Disable PIB interrupts at the VIC. */
    CyFx3BootGpifUnregisterIsr ();

    /* Clear and disable all interrupts. */
    PMMC->intr_mask = 0x00000000;
    PMMC->intr      = 0xFFFFFFFF;
    PIB->intr_mask  = 0x00000000;
    PIB->intr       = 0xFFFFFFFF;

    /* Power the block off and stop the clock. */
    CyFx3PibPowerOff ();
    CyFx3BootBusyWait (10);
    GCTL->pib_core_clk &= ~CY_U3P_GCTL_PIBCLK_CLK_EN;

    glBootPibStarted = CyFalse;
    glBootPibMmcMode = CyFalse;

    return CY_FX3_BOOT_SUCCESS;
}

void
CyFx3BootPibRegisterMmcCallback (
        CyFx3BootPMMCIntrCb_t cb)
{
    glBootPibMmcCallback = cb;
}

void
CyFx3BootPibHandleEvents (
        void)
{
    uint32_t pib_intstat, pib_intmask;
    uint32_t mmc_intstat, mmc_intmask;
    uint32_t v1, v2;
    uint8_t  index, value;

    pib_intmask = PIB->intr_mask;
    pib_intstat = PIB->intr;
    v1          = pib_intmask & pib_intstat;

    if (v1 != 0)
    {
        /* Clear the interrupt. */
        PIB->intr = v1;

        if ((v1 & CY_U3P_PIB_INTR_PMMC_INTR1) != 0)
        {

            mmc_intmask = PMMC->intr_mask;
            mmc_intstat = PMMC->intr;
            v2          = mmc_intmask & mmc_intstat;

            if (v2 != 0)
            {
                /* Clear all interrupts that are active. */
                PMMC->intr = v2;

                if (!glBootPibMmcMode)
                    return;

                if ((v2 & CY_U3P_PIB_PMMC_GO_IDLE) != 0)
                {
                    if (glBootPibMmcCallback != 0)
                        glBootPibMmcCallback (CYFX3_PMMC_GOIDLE_CMD, 0);
                }

                if ((v2 & CY_U3P_PIB_PMMC_CMD1) != 0)
                {
                    /* Make sure the ready bit is set. */
                    PMMC->ocr |= CY_U3P_PIB_PMMC_OCR_STATUS;
                }

                if ((v2 & CY_U3P_PIB_PMMC_CMD5_SLEEP) != 0)
                {
                    if (glBootPibMmcCallback != 0)
                        glBootPibMmcCallback (CYFX3_PMMC_CMD5_SLEEP, 0);
                }

                if ((v2 & CY_U3P_PIB_PMMC_CMD5_AWAKE) != 0)
                {
                    if (glBootPibMmcCallback != 0)
                        glBootPibMmcCallback (CYFX3_PMMC_CMD5_AWAKE, 0);
                }

                if ((v2 & CY_U3P_PIB_PMMC_CMD6) != 0)
                {
                    if (glBootPibMmcCallback != 0)
                        glBootPibMmcCallback (CYFX3_PMMC_CMD6_SWITCH, PMMC->arg);

                    index = (uint8_t)((PMMC->arg & (0xFF << 16)) >> 16);
                    value = (uint8_t)((PMMC->arg & (0xFF << 8)) >> 8);

                    switch ((PMMC->arg & 0x03000000) >> 24)
                    {
                        case 0: /* Switch command set */
                            /* Do not Support swich command set */
                            PMMC->csr |= CY_U3P_PIB_CSR_SWITCH_ERROR;
                            break;

                        case 3: /* Write byte */
                            switch (index)
                            {
                                case 183:       /* Bus Width. */
                                    if (value < 3)
                                    {
                                        /* Update the bus width as requested. */
                                        PMMC->ext_csd0 = (PMMC->ext_csd0 & ~CY_U3P_PIB_EXT_CSD0_BUS_WIDTH_MASK) | value;
                                    }
                                    break;
                                case 185:       /* HS timing. */
                                    if (value != 0)
                                        PMMC->ext_csd0 |= (0x01 << CY_U3P_PIB_EXT_CSD0_HS_TIMING_POS);
                                    else
                                        PMMC->ext_csd0 &= ~CY_U3P_PIB_EXT_CSD0_HS_TIMING_MASK;
                                    break;
                                default:
                                    PMMC->csr |= CY_U3P_PIB_CSR_SWITCH_ERROR;
                                    break;
                            }

                        default: /* Set bits or Clear bits*/
                            if (index == 185)
                            {
                                if (value != 0)
                                    PMMC->ext_csd0 |= (0x01 << CY_U3P_PIB_EXT_CSD0_HS_TIMING_POS);
                                else
                                    PMMC->ext_csd0 &= ~CY_U3P_PIB_EXT_CSD0_HS_TIMING_MASK;
                            }
                            else
                                PMMC->csr |= CY_U3P_PIB_CSR_SWITCH_ERROR;
                            break;
                    }

                    /* Clear the busy bit */
                    PMMC->busy = CY_U3P_PIB_PMMC_BUSY_CLEAR;
                }

                if ((v2 & CY_U3P_PIB_PMMC_CMD12) != 0)
                {
                    if (glBootPibMmcCallback != 0)
                        glBootPibMmcCallback (CYFX3_PMMC_CMD12_STOP, PMMC->blk_addr);
                }

                if ((v2 & CY_U3P_PIB_PMMC_NEW_CMD_RCVD) != 0)
                {
                    /* Notify application if select/deselect is performed. */
                    if ((PMMC->idx & CY_U3P_PIB_PMMC_CMD_MASK) == 7)
                    {
                        if (glBootPibMmcCallback != 0)
                            glBootPibMmcCallback (CYFX3_PMMC_CMD7_SELECT, PMMC->arg);
                    }
                }

                if ((v2 & (CY_U3P_PIB_PMMC_WR_SCK_NOT_RDY | CY_U3P_PIB_PMMC_RD_SCK_NOT_RDY)) != 0)
                {
                    if (glBootPibMmcCallback != 0)
                        glBootPibMmcCallback (CYFX3_PMMC_SOCKET_NOT_READY, PMMC->blk_addr);
                }

                if ((v2 & CY_U3P_PIB_PMMC_CMD15) != 0)
                {
                    if (glBootPibMmcCallback != 0)
                        glBootPibMmcCallback (CYFX3_PMMC_CMD15_INACTIVE, 0);
                }
            }
        }
    }
}

static void
CyFx3BootPibDelayFunc (
        uint16_t usWait)
{
    CyFx3BootPibHandleEvents ();
    CyFx3BootBusyWait (usWait);
}

CyFx3BootErrorCode_t
CyFx3BootPibDmaXferData (
        uint8_t  sockNum,
        CyBool_t isRead,
        uint32_t address,
        uint32_t length,
        uint32_t timeout)
{
    PSCK_T   s;
    PDSCR_T  dscr = DSCR (CY_FX3_PIB_DMA_DSCR_INDEX);
    uint32_t sync, size;

    CyFx3BootErrorCode_t status;

    /* Parameter checks. */
    if ((address < CY_FX3_BOOT_SYSMEM_BASE) || (address >= CY_FX3_BOOT_SYSMEM_END))
        return CY_FX3_BOOT_ERROR_INVALID_DMA_ADDR;
    if ((sockNum >= 32) || (length == 0) || ((length & 0x0F) != 0) || (length > CYFX3_DMA_MAX_SIZE))
        return CY_FX3_BOOT_ERROR_BAD_ARGUMENT;

    /* Pick the socket to be configured. */
    s = (PSCK_T)&PIB->sck[sockNum];

    /* Wait until the socket is disabled. */
    s->status = CY_U3P_LPP_SCK_STATUS_DEFAULT;
    while (s->status & CY_U3P_LPP_ENABLED)
       __nop ();

    /* Configure the socket as required. */
    s->status = CY_U3P_LPP_SCK_STATUS_DEFAULT | CY_U3P_LPP_UNIT;        /* Use buffer as unit for transfer size. */
    s->size   = 1;                                                      /* Single buffer transfer. */
    s->count  = 0;                                                      /* Clear count at the beginning. */
    s->intr   = 0xFF;                                                   /* Clear all interrupts */
    s->dscr   = DSCR_ADDR(dscr);                                        /* Address of the descriptor. */

    if (!isRead)
    {
        /* Write from FX3 firmware => Egress Transfer. */
        sync = ((sockNum << CY_U3P_LPP_CONS_SCK_POS) | (CYFX3_PIB_DMA_ID << CY_U3P_LPP_CONS_IP_POS) |
                (CPU_SCK_NUM << CY_U3P_LPP_PROD_SCK_POS) | (CPU_IP_NUM << CY_U3P_LPP_PROD_IP_POS) |
                CY_U3P_LPP_EN_CONS_EVENT | CY_U3P_LPP_EN_CONS_INT | CY_U3P_LPP_EN_PROD_EVENT | CY_U3P_LPP_EN_PROD_INT
               );
        size = (length << CY_U3P_LPP_BYTE_COUNT_POS) | length | CY_U3P_LPP_BUFFER_OCCUPIED;
    }
    else
    {
        /* Read into FX3 => Ingress Transfer. */
        sync = ((CPU_SCK_NUM << CY_U3P_LPP_CONS_SCK_POS) | (CPU_IP_NUM << CY_U3P_LPP_CONS_IP_POS) |
                (sockNum << CY_U3P_LPP_PROD_SCK_POS) | (CYFX3_PIB_DMA_ID << CY_U3P_LPP_PROD_IP_POS) |
                CY_U3P_LPP_EN_CONS_EVENT | CY_U3P_LPP_EN_CONS_INT | CY_U3P_LPP_EN_PROD_EVENT | CY_U3P_LPP_EN_PROD_INT
               );
        size = length;
    }

    /* Configure the descriptor. */
    dscr->buffer = address;
    dscr->sync   = sync;
    dscr->size   = size;
    dscr->chain  = (0xFFFFFFFF);

    status = CY_FX3_BOOT_ERROR_TIMEOUT;

    /* Enable the socket and wait for the transfer to complete. */
    s->status |= CY_U3P_LPP_GO_ENABLE;
    do
    {
        /* If the socket reports an error, abort. */
        if (s->intr & CY_U3P_LPP_ERROR)
        {
            status = CY_FX3_BOOT_ERROR_XFER_FAILURE;
            break;
        }

        if (!isRead)
        {
            /* Data has been sent out. Return success. */
            if (s->intr & CY_U3P_LPP_CONSUME_EVENT)
            {
                status = CY_FX3_BOOT_SUCCESS;
                break;
            }
        }
        else
        {
            /* Data has been received. Return success. */
            if (s->intr & CY_U3P_LPP_PRODUCE_EVENT)
            {
                status = CY_FX3_BOOT_SUCCESS;
                break;
            }
        }

        CyFx3BootPibDelayFunc (100);
        if ((timeout != CY_FX3_BOOT_NO_WAIT) && (timeout != CY_FX3_BOOT_WAIT_FOREVER))
        {
            timeout--;
        }
    } while (timeout > 0);

    return status;
}

void
CyFx3BootGpifRegisterCallback (
        CyFx3BootGpifIntrCb_t cbFunc)
{
    glBootGpifCallback = cbFunc;
}

CyFx3BootErrorCode_t
CyFx3BootGpifLoad (
        const CyU3PGpifConfig_t *conf)
{
    uint16_t index, entry;
    uvint32_t *addr = (uvint32_t *)CY_U3P_PIB_GPIF_BUS_CONFIG_ADDRESS;

    /* PIB state sanity checks. */
    if (!glBootPibStarted)
        return CY_FX3_BOOT_ERROR_NOT_STARTED;
    if (glBootPibMmcMode)
        return CY_FX3_BOOT_ERROR_NOT_CONFIGURED;

    /* Parameter checks. */
    if ((conf == 0) || ((conf->stateCount == 0) && (conf->functionCount == 0) && (conf->regCount == 0)))
    {
        return CY_FX3_BOOT_ERROR_BAD_ARGUMENT;
    }


    /* Verify that the GPIF has not been enabled yet. */
    if ((PIB->gpif_config & CY_U3P_GPIF_CONF_ENABLE) || (PIB->gpif_waveform_ctrl_stat & CY_U3P_GPIF_WAVEFORM_VALID))
    {
        return CY_FX3_BOOT_ERROR_ALREADY_STARTED;
    }

    if (!CyFx3DevIsGpif32Supported ())
    {
        if ((conf->regCount >= 2) && (conf->regData != 0) &&
                ((conf->regData[1] & CY_U3P_GPIF_BUS_WIDTH_MASK) > 0x04))
            return CY_FX3_BOOT_ERROR_NOT_SUPPORTED;
    }

    if (conf->stateCount)
    {
        if (conf->stateData == 0)
            return CY_FX3_BOOT_ERROR_BAD_ARGUMENT;

        for (index = 0; index < conf->stateCount; index++)
        {
            /* No look-up table means direct mapping from array index to state number. */
            entry = (conf->statePosition != 0) ? conf->statePosition[index] : index;

            PIB->gpif_left[index].waveform0  = conf->stateData[entry].leftData[0];
            PIB->gpif_left[index].waveform1  = conf->stateData[entry].leftData[1];
            PIB->gpif_left[index].waveform2  = conf->stateData[entry].leftData[2];

            PIB->gpif_right[index].waveform0 = conf->stateData[entry].rightData[0];
            PIB->gpif_right[index].waveform1 = conf->stateData[entry].rightData[1];
            PIB->gpif_right[index].waveform2 = conf->stateData[entry].rightData[2];
        }
    }

    if (conf->functionCount)
    {
        if ((conf->functionData == 0) || (conf->functionCount > CYFX3_GPIF_NUM_TRANS_FNS))
            return CY_FX3_BOOT_ERROR_BAD_ARGUMENT;
        for (index = 0; index < conf->functionCount; index++)
            PIB->gpif_function[index] = conf->functionData[index];
    }

    if (conf->regCount)
    {
        if (conf->regData == 0)
            return CY_FX3_BOOT_ERROR_BAD_ARGUMENT;

        for (index = 1; index < conf->regCount; index++)
        {
            /* Make sure only the GPIF SM interrupt is turned on. */
            if (index == 6)
                *addr++ = conf->regData[index] & CY_U3P_GPIF_INTR_GPIF_INTR;
            else
                *addr++ = conf->regData[index];
        }

        /* The GPIF config register is initialized at the end. */
        PIB->gpif_config = conf->regData[0];
    }

    return CY_FX3_BOOT_SUCCESS;
}

#define GPIF_CONF_REG_COUNT (76)
const uint32_t glConstGpifDefaults[GPIF_CONF_REG_COUNT] = {
    CY_U3P_PIB_GPIF_CONFIG_DEFAULT,
    CY_U3P_PIB_GPIF_BUS_CONFIG_DEFAULT,
    CY_U3P_PIB_GPIF_BUS_CONFIG2_DEFAULT,
    CY_U3P_PIB_GPIF_AD_CONFIG_DEFAULT,
    CY_U3P_PIB_GPIF_STATUS_DEFAULT,
    CY_U3P_PIB_GPIF_INTR_DEFAULT,
    CY_U3P_PIB_GPIF_INTR_MASK_DEFAULT,
    CY_U3P_PIB_GPIF_SERIAL_IN_CONFIG_DEFAULT,
    CY_U3P_PIB_GPIF_SERIAL_OUT_CONFIG_DEFAULT,
    CY_U3P_PIB_GPIF_CTRL_BUS_DIRECTION_DEFAULT,
    CY_U3P_PIB_GPIF_CTRL_BUS_DEFAULT_DEFAULT,
    CY_U3P_PIB_GPIF_CTRL_BUS_POLARITY_DEFAULT,
    CY_U3P_PIB_GPIF_CTRL_BUS_TOGGLE_DEFAULT,
    CY_U3P_PIB_GPIF_CTRL_BUS_SELECT_DEFAULT,
    CY_U3P_PIB_GPIF_CTRL_BUS_SELECT_DEFAULT,
    CY_U3P_PIB_GPIF_CTRL_BUS_SELECT_DEFAULT,
    CY_U3P_PIB_GPIF_CTRL_BUS_SELECT_DEFAULT,
    CY_U3P_PIB_GPIF_CTRL_BUS_SELECT_DEFAULT,
    CY_U3P_PIB_GPIF_CTRL_BUS_SELECT_DEFAULT,
    CY_U3P_PIB_GPIF_CTRL_BUS_SELECT_DEFAULT,
    CY_U3P_PIB_GPIF_CTRL_BUS_SELECT_DEFAULT,
    CY_U3P_PIB_GPIF_CTRL_BUS_SELECT_DEFAULT,
    CY_U3P_PIB_GPIF_CTRL_BUS_SELECT_DEFAULT,
    CY_U3P_PIB_GPIF_CTRL_BUS_SELECT_DEFAULT,
    CY_U3P_PIB_GPIF_CTRL_BUS_SELECT_DEFAULT,
    CY_U3P_PIB_GPIF_CTRL_BUS_SELECT_DEFAULT,
    CY_U3P_PIB_GPIF_CTRL_BUS_SELECT_DEFAULT,
    CY_U3P_PIB_GPIF_CTRL_BUS_SELECT_DEFAULT,
    CY_U3P_PIB_GPIF_CTRL_BUS_SELECT_DEFAULT,
    CY_U3P_PIB_GPIF_CTRL_COUNT_CONFIG_DEFAULT,
    CY_U3P_PIB_GPIF_CTRL_COUNT_RESET_DEFAULT,
    CY_U3P_PIB_GPIF_CTRL_COUNT_LIMIT_DEFAULT,
    CY_U3P_PIB_GPIF_ADDR_COUNT_CONFIG_DEFAULT,
    CY_U3P_PIB_GPIF_ADDR_COUNT_RESET_DEFAULT,
    CY_U3P_PIB_GPIF_ADDR_COUNT_LIMIT_DEFAULT,
    CY_U3P_PIB_GPIF_STATE_COUNT_CONFIG_DEFAULT,
    CY_U3P_PIB_GPIF_STATE_COUNT_LIMIT_DEFAULT,
    CY_U3P_PIB_GPIF_DATA_COUNT_CONFIG_DEFAULT,
    CY_U3P_PIB_GPIF_DATA_COUNT_RESET_DEFAULT,
    CY_U3P_PIB_GPIF_DATA_COUNT_LIMIT_DEFAULT,
    CY_U3P_PIB_GPIF_CTRL_COMP_VALUE_DEFAULT,
    CY_U3P_PIB_GPIF_CTRL_COMP_MASK_DEFAULT,
    CY_U3P_PIB_GPIF_DATA_COMP_VALUE_DEFAULT,
    CY_U3P_PIB_GPIF_DATA_COMP_MASK_DEFAULT,
    CY_U3P_PIB_GPIF_ADDR_COMP_VALUE_DEFAULT,
    CY_U3P_PIB_GPIF_ADDR_COMP_MASK_DEFAULT,
    CY_U3P_PIB_GPIF_DATA_CTRL_DEFAULT,
    CY_U3P_PIB_GPIF_INGRESS_DATA_DEFAULT,
    CY_U3P_PIB_GPIF_INGRESS_DATA_DEFAULT,
    CY_U3P_PIB_GPIF_INGRESS_DATA_DEFAULT,
    CY_U3P_PIB_GPIF_INGRESS_DATA_DEFAULT,
    CY_U3P_PIB_GPIF_EGRESS_DATA_DEFAULT,
    CY_U3P_PIB_GPIF_EGRESS_DATA_DEFAULT,
    CY_U3P_PIB_GPIF_EGRESS_DATA_DEFAULT,
    CY_U3P_PIB_GPIF_EGRESS_DATA_DEFAULT,
    CY_U3P_PIB_GPIF_INGRESS_ADDRESS_DEFAULT,
    CY_U3P_PIB_GPIF_INGRESS_ADDRESS_DEFAULT,
    CY_U3P_PIB_GPIF_INGRESS_ADDRESS_DEFAULT,
    CY_U3P_PIB_GPIF_INGRESS_ADDRESS_DEFAULT,
    CY_U3P_PIB_GPIF_EGRESS_ADDRESS_DEFAULT,
    CY_U3P_PIB_GPIF_EGRESS_ADDRESS_DEFAULT,
    CY_U3P_PIB_GPIF_EGRESS_ADDRESS_DEFAULT,
    CY_U3P_PIB_GPIF_EGRESS_ADDRESS_DEFAULT,
    CY_U3P_PIB_GPIF_THREAD_CONFIG_DEFAULT,
    CY_U3P_PIB_GPIF_THREAD_CONFIG_DEFAULT,
    CY_U3P_PIB_GPIF_THREAD_CONFIG_DEFAULT,
    CY_U3P_PIB_GPIF_THREAD_CONFIG_DEFAULT,
    CY_U3P_PIB_GPIF_LAMBDA_STAT_DEFAULT,
    CY_U3P_PIB_GPIF_ALPHA_STAT_DEFAULT,
    CY_U3P_PIB_GPIF_BETA_STAT_DEFAULT,
    CY_U3P_PIB_GPIF_WAVEFORM_CTRL_STAT_DEFAULT,
    CY_U3P_PIB_GPIF_WAVEFORM_SWITCH_DEFAULT,
    CY_U3P_PIB_GPIF_WAVEFORM_SWITCH_TIMEOUT_DEFAULT,
    CY_U3P_PIB_GPIF_CRC_CONFIG_DEFAULT,
    CY_U3P_PIB_GPIF_CRC_DATA_DEFAULT,
    CY_U3P_PIB_GPIF_BETA_DEASSERT_DEFAULT
};

void
CyFx3BootGpifDisable (
        CyBool_t forceReload)
{
    uint32_t   i;
    uvint32_t *addr = (uvint32_t *)CY_U3P_PIB_GPIF_BUS_CONFIG_ADDRESS;

    /* If PIB is not started, or is in MMC mode; do nothing. */
    if ((!glBootPibStarted) || (glBootPibMmcMode))
        return;

    /* Pause the state machine to stop operation and then disable it. */
    PIB->gpif_waveform_ctrl_stat |= CY_U3P_GPIF_PAUSE;
    CyFx3BootBusyWait (10);
    PIB->gpif_waveform_ctrl_stat  = CY_U3P_PIB_GPIF_WAVEFORM_CTRL_STAT_DEFAULT;

    if (forceReload)
    {
        PIB->gpif_config = CY_U3P_PIB_GPIF_CONFIG_DEFAULT;

        /* Clear all existing state information from the waveform memory. */
        for (i = 0; i < CYFX3_GPIF_NUM_STATES; i++)
        {
            PIB->gpif_left[i].waveform2  = 0;
            PIB->gpif_right[i].waveform2 = 0;
        }

        /* Reset all GPIF registers to their default values. */
        for (i = 1; i < GPIF_CONF_REG_COUNT; i++)
        {
            *addr++ = glConstGpifDefaults[i];
        }

        PIB->gpif_config = glConstGpifDefaults[0];
    }
}

CyFx3BootErrorCode_t
CyFx3BootGpifSMStart (
        uint8_t startState,
        uint8_t initialAlpha)
{
    uint32_t numdss;
    uint32_t gpifstat;
    uint32_t switchVal;

    /* PIB state sanity checks. */
    if (!glBootPibStarted)
        return CY_FX3_BOOT_ERROR_NOT_STARTED;
    if (glBootPibMmcMode)
        return CY_FX3_BOOT_ERROR_NOT_CONFIGURED;

    /* State checks. */
    if (((PIB->gpif_config & CY_U3P_GPIF_CONF_ENABLE) == 0) ||
            ((PIB->gpif_left[startState].waveform2 & CY_U3P_GPIF2_VALID) == 0))
    {
        return CY_FX3_BOOT_ERROR_NOT_CONFIGURED;
    }

    if (PIB->gpif_waveform_ctrl_stat & CY_U3P_GPIF_WAVEFORM_VALID)
    {
        return CY_FX3_BOOT_ERROR_ALREADY_STARTED;
    }

    /* Mark the waveform as valid. */
    CY_U3P_PIB_GPIF_WAVEFORM_CTRL_STAT = ((CY_U3P_PIB_GPIF_WAVEFORM_CTRL_STAT & ~CY_U3P_GPIF_ALPHA_INIT_MASK) |
            (initialAlpha << CY_U3P_GPIF_ALPHA_INIT_POS) | CY_U3P_GPIF_WAVEFORM_VALID);

    /* Switch to the desired state and start execution. */
    switchVal = CY_U3P_PIB_GPIF_WAVEFORM_SWITCH;
    switchVal = ((switchVal & 0xFF00003A) | (startState << CY_U3P_GPIF_DESTINATION_STATE_POS));

    CY_U3P_PIB_GPIF_WAVEFORM_SWITCH  = switchVal;
    CY_U3P_PIB_GPIF_WAVEFORM_SWITCH |= (CY_U3P_GPIF_WAVEFORM_SWITCH | CY_U3P_GPIF_SWITCH_NOW);

    gpifstat = CY_U3P_PIB_GPIF_WAVEFORM_CTRL_STAT;
    numdss   = (CY_U3P_PIB_GPIF_BUS_CONFIG2 & CY_U3P_GPIF_STATE_FROM_CTRL_MASK);
    switch (numdss)
    {
    case 1:
        CY_U3P_PIB_GPIF_WAVEFORM_SWITCH = ((switchVal & 0xFF7F003A) | CY_U3P_GPIF_WAVEFORM_SWITCH |
                CY_U3P_GPIF_SWITCH_NOW | ((gpifstat & 0x80000000u) >> 8));
        break;
    case 2:
        CY_U3P_PIB_GPIF_WAVEFORM_SWITCH = ((switchVal & 0xFF3F003A) | CY_U3P_GPIF_WAVEFORM_SWITCH |
                CY_U3P_GPIF_SWITCH_NOW | ((gpifstat & 0xC0000000u) >> 8));
        break;
    case 3:
        CY_U3P_PIB_GPIF_WAVEFORM_SWITCH = ((switchVal & 0xFF1F003A) | CY_U3P_GPIF_WAVEFORM_SWITCH |
                CY_U3P_GPIF_SWITCH_NOW | ((gpifstat & 0xE0000000u) >> 8));
        break;
    default:
        break;
    }

    return CY_FX3_BOOT_SUCCESS;

}

/* Maximum number of distinct states in the GPIF state machine (DSS dependent). */
const uint8_t glGpifConstMaxState[4] = {
    0xFF, 0x7F, 0x3F, 0x1F
};

CyFx3BootErrorCode_t
CyFx3BootGpifSMSwitch (
        uint16_t fromState,
        uint16_t toState,
        uint8_t  initialAlpha)
{
    uint32_t switchVal = 0;
    uint32_t numdss;
    uint32_t gpifstat;
    uint8_t  curState;

    /* PIB state sanity checks. */
    if (!glBootPibStarted)
        return CY_FX3_BOOT_ERROR_NOT_STARTED;
    if (glBootPibMmcMode)
        return CY_FX3_BOOT_ERROR_NOT_CONFIGURED;

    /* Parameter and state checks. */
    if (toState >= CYFX3_GPIF_NUM_STATES)
    {
        return CY_FX3_BOOT_ERROR_BAD_ARGUMENT;
    }

    /* Removing the condition check for valid state because it is not possible to read GPIF
       waveform registers while GPIF is running. */
    if ((PIB->gpif_config & CY_U3P_GPIF_CONF_ENABLE) == 0)
    {
        return CY_FX3_BOOT_ERROR_NOT_CONFIGURED;
    }

    /* Find the DSS level of the state machine. */
    numdss = (CY_U3P_PIB_GPIF_BUS_CONFIG2 & CY_U3P_GPIF_STATE_FROM_CTRL_MASK);

    /* Update the initial Alpha values for the next state switch. */
    CY_U3P_PIB_GPIF_WAVEFORM_CTRL_STAT = ((CY_U3P_PIB_GPIF_WAVEFORM_CTRL_STAT & ~CY_U3P_GPIF_ALPHA_INIT_MASK) |
            (initialAlpha << CY_U3P_GPIF_ALPHA_INIT_POS) | CY_U3P_GPIF_WAVEFORM_VALID);

    switchVal = (toState << CY_U3P_GPIF_DESTINATION_STATE_POS) | CY_U3P_GPIF_WAVEFORM_SWITCH;

    if (fromState < CYFX3_GPIF_NUM_STATES)
    {
        /* Check if the state machine is already in the fromState or a mirror. */
        curState = (PIB->gpif_waveform_ctrl_stat & CY_U3P_GPIF_CURRENT_STATE_MASK) >> CY_U3P_GPIF_CURRENT_STATE_POS;
        if ((curState & glGpifConstMaxState[numdss]) == (fromState & glGpifConstMaxState[numdss]))
        {
            switchVal |= CY_U3P_GPIF_SWITCH_NOW;
        }
        else
        {
            /* The timeout setting is assumed to be valid if a fromState is specified. */
            switchVal |= (fromState << CY_U3P_GPIF_TERMINAL_STATE_POS);
        }
    }
    else
    {
        /* Set the switch immediately flag. */
        switchVal |= CY_U3P_GPIF_SWITCH_NOW;
    }

    /* Write into the GPIF register to request the switch. */
    PIB->gpif_waveform_switch = switchVal;

    gpifstat = CY_U3P_PIB_GPIF_WAVEFORM_CTRL_STAT;
    switch (numdss)
    {
    case 1:
        CY_U3P_PIB_GPIF_WAVEFORM_SWITCH = ((switchVal & 0xFF7FFFFF) | ((gpifstat & 0x80000000u) >> 8));
        break;
    case 2:
        CY_U3P_PIB_GPIF_WAVEFORM_SWITCH = ((switchVal & 0xFF3FFFFF) | ((gpifstat & 0xC0000000u) >> 8));
        break;
    case 3:
        CY_U3P_PIB_GPIF_WAVEFORM_SWITCH = ((switchVal & 0xFF1FFFFF) | ((gpifstat & 0xE0000000u) >> 8));
        break;
    default:
        break;
    }

    return CY_FX3_BOOT_SUCCESS;
}

CyFx3BootErrorCode_t
CyFx3BootGpifInitCounter (
        CyFx3GpifCounterType counter,
        uint32_t             initValue,
        uint32_t             limit,
        CyBool_t             reload,
        int8_t               increment,
        uint8_t              outputbit)
{
    uint32_t cfgVal = 0;

    /* PIB state sanity checks. */
    if ((!glBootPibStarted) || (glBootPibMmcMode))
        return CY_FX3_BOOT_ERROR_NOT_STARTED;

    switch (counter)
    {
        case CYFX3_GPIF_COUNTER_CONTROL:

            /* We can only update this counter by one at a time. */
            if (((increment != 1) && (increment != -1)) || (initValue > 0xFFFF) || (limit > 0xFFFF))
                return CY_FX3_BOOT_ERROR_BAD_ARGUMENT;

            /* Make sure that the counter is stopped. */
            PIB->gpif_ctrl_count_config = 0;

            /* Set the initial value and the limit. */
            PIB->gpif_ctrl_count_reset = initValue;
            PIB->gpif_ctrl_count_limit = limit;

            /* Set the counter configuration as desired. */
            cfgVal = CY_U3P_GPIF_CC_SW_RESET | CY_U3P_GPIF_CC_ENABLE |
                ((outputbit << CY_U3P_GPIF_CC_CONNECT_POS) & CY_U3P_GPIF_CC_CONNECT_MASK);
            if (reload)
            {
                cfgVal |= CY_U3P_GPIF_CC_RELOAD;
            }
            if (increment > 0)
            {
                cfgVal |= CY_U3P_GPIF_CC_DOWN_UP;
            }

            PIB->gpif_ctrl_count_config = cfgVal;
            break;

        case CYFX3_GPIF_COUNTER_ADDRESS:

            if (increment == 0)
                return CY_FX3_BOOT_ERROR_BAD_ARGUMENT;

            /* Make sure that the counter is stopped. */
            PIB->gpif_addr_count_config = 0;

            /* Set the initial value and the limit. */
            PIB->gpif_addr_count_reset = initValue;
            PIB->gpif_addr_count_limit = limit;

            if (increment < 0)
            {
                increment = (int8_t)(-increment);
            }
            else
            {
                cfgVal = CY_U3P_GPIF_AC_DOWN_UP;
            }

            /* Set the counter configuration as desired. */
            cfgVal |= (increment <<  CY_U3P_GPIF_AC_INCREMENT_POS) | CY_U3P_GPIF_AC_SW_RESET | CY_U3P_GPIF_AC_ENABLE;
            if (reload)
            {
                cfgVal |= CY_U3P_GPIF_AC_RELOAD;
            }

            PIB->gpif_addr_count_config = cfgVal;
            break;

        case CYFX3_GPIF_COUNTER_DATA:

            if (increment == 0)
                return CY_FX3_BOOT_ERROR_BAD_ARGUMENT;

            /* Make sure that the counter is stopped. */
            PIB->gpif_data_count_config = 0;

            /* Set the initial value and the limit. */
            PIB->gpif_data_count_reset = initValue;
            PIB->gpif_data_count_limit = limit;

            if (increment < 0)
            {
                increment = (int8_t)(-increment);
            }
            else
            {
                cfgVal = CY_U3P_GPIF_AC_DOWN_UP;
            }

            /* Set the counter configuration as desired. */
            cfgVal |= (increment <<  CY_U3P_GPIF_DC_INCREMENT_POS) | CY_U3P_GPIF_DC_SW_RESET | CY_U3P_GPIF_DC_ENABLE;
            if (reload)
            {
                cfgVal |= CY_U3P_GPIF_DC_RELOAD;
            }

            PIB->gpif_data_count_config = cfgVal;
            break;

        default:
            break;
    }

    return CY_FX3_BOOT_SUCCESS;
}

void
CyFx3BootGpifControlSWInput (
        CyBool_t set)
{
    /* PIB state sanity checks. */
    if ((!glBootPibStarted) || (glBootPibMmcMode))
        return;

    if (set)
    {
        CY_U3P_PIB_GPIF_WAVEFORM_CTRL_STAT |= CY_U3P_GPIF_CPU_LAMBDA;
    }
    else
    {
        CY_U3P_PIB_GPIF_WAVEFORM_CTRL_STAT &= ~CY_U3P_GPIF_CPU_LAMBDA;
    }
}

uint8_t
CyFx3BootGpifGetState (
        void)
{
    if ((glBootPibStarted) && (!glBootPibMmcMode))
    {
        return (uint8_t)((PIB->gpif_waveform_ctrl_stat & CY_U3P_GPIF_CURRENT_STATE_MASK) >>
                CY_U3P_GPIF_CURRENT_STATE_POS);
    }

    return 0;
}

CyFx3BootErrorCode_t
CyFx3BootGpifSocketConfigure (
        uint8_t  threadIndex,
        uint8_t  socketNum,
        uint16_t watermark,
        CyBool_t flagOnData,
        uint8_t  burst)
{
    uint32_t regVal = 0;

    if (!glBootPibStarted)
        return CY_FX3_BOOT_ERROR_NOT_STARTED;

    /* Parameter checks. */
    if ((threadIndex >= CYFX3_PIB_THREAD_COUNT) || (burst > CYFX3_PIB_MAX_BURST_SETTING) ||
            (socketNum > CYFX3_PIB_SOCKET_COUNT))
    {
        return CY_FX3_BOOT_ERROR_BAD_ARGUMENT;
    }

    /* In PP-mode, only thread 0 can be used. */
    if (((PIB->gpif_config & CY_U3P_GPIF_CONF_PP_MODE) != 0) && (threadIndex != 0))
    {
        return CY_FX3_BOOT_ERROR_BAD_ARGUMENT;
    }

    /* Disable the thread before enabling it with the correct configuration. */
    PIB->gpif_thread_config[threadIndex] &= ~CY_U3P_GPIF_ENABLE;
    CyFx3BootBusyWait (1);

    /* Compute and update the register value. */
    regVal = socketNum | (burst << CY_U3P_GPIF_BURST_SIZE_POS) |
        ((watermark << CY_U3P_GPIF_WATERMARK_POS) & CY_U3P_GPIF_WATERMARK_MASK) | CY_U3P_GPIF_ENABLE;
    if (flagOnData)
    {
        regVal |= CY_U3P_GPIF_WM_CFG;
    }

    PIB->gpif_thread_config[threadIndex] = regVal;
    return CY_FX3_BOOT_SUCCESS;
}

/*[]*/

