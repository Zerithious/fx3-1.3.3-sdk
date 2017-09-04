/*
 ## Cypress FX3 Device Firmware Source (cyu3system.c)
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

/* This file defines the system thread and the various device specific
 * initializations
 */

#include <cyu3system.h>
#include <cyu3os.h>
#include <cyu3socket.h>
#include <cyu3dma.h>
#include <cyu3protocol.h>
#include <cyu3sib.h>
#include <cyu3error.h>
#include <cyu3vic.h>
#include <cyu3appdefine.h>
#include <cyu3standby.h>
#include <cyu3regs.h>
#include <cyu3utils.h>
#include <cyu3mmu.h>
#include <cyu3usb.h>
#include <cyu3lpp.h>
#include <cyu3usbotg.h>
#include <cyu3usbhost.h>
#include <cyu3mbox.h>
#include <cyfx3_api.h>

#ifdef CYU3P_PROFILE_EN

#define MAX_DRIVER_THREAD       (16)
static CyU3PThread *glFx3DrvThreads[MAX_DRIVER_THREAD] = {0};           /* List of driver threads. */
static uint32_t     glFx3DrvCount = 0;                                  /* Number of driver threads. */

#endif /* CYU3P_PROFILE_EN */

#include <cyfxversion.h>
const uint16_t cyfx_version_major = CYFX_VERSION_MAJOR;
const uint16_t cyfx_version_minor = CYFX_VERSION_MINOR;
const uint16_t cyfx_version_patch = CYFX_VERSION_PATCH;
const uint16_t cyfx_version_build = CYFX_VERSION_BUILD;

#define CY_U3P_SYS_THREAD_STACK         (0x400)
#define CY_U3P_SYS_THREAD_PRIORITY      (4)

#define CY_U3P_ITCM_BASE                (0x00000000)
#define CY_U3P_ITCM_SIZE                (0x4000)
#define CY_U3P_DTCM_BASE                (0x10000000)
#define CY_U3P_DTCM_SIZE                (0x2000)

CyU3PThread     glSysThread;
CyU3PEvent      glSysEvent;

static CyU3PThread  *glTmpThread;
static uint16_t     glRegFlag = 0;
static unsigned int  glThreadPriority;
static uint32_t     glIntrEnabled = 0;
static CyBool_t     glIsSysLPModeParamChkd = CyFalse;

#define CyU3PSysGetLPModeStatus()        (CyBool_t) (glIsSysLPModeParamChkd)
#define CyU3PSysSetLPModeStatus(status)  (glIsSysLPModeParamChkd = (status))

#define FX3_ACTIVE_SIG          (0x46583320)    /* "FX3 " */
#define FX3_STANDBY_SIG         (0x53544259)    /* "STBY" */

struct CyFx3BootState {
    uint32_t  signature;
    uint32_t  isWarmBoot;
    uint32_t *regBkpAddr;
};

struct CyFx3BootState glDevBootState = {
    FX3_ACTIVE_SIG, 0, 0
};

extern uint8_t glLppActive;

/* This API is intended only for FX3. */
CyU3PReturnStatus_t
CyU3PSysGetApiVersion (
        uint16_t *majorVersion,
        uint16_t *minorVersion,
        uint16_t *patchNumber,
        uint16_t *buildNumber)
{
    if (majorVersion != NULL)
    {
        *majorVersion = cyfx_version_major;
    }
    if (minorVersion != NULL)
    {
        *minorVersion = cyfx_version_minor;
    }
    if (patchNumber != NULL)
    {
        *patchNumber = cyfx_version_patch;
    }
    if (buildNumber != NULL)
    {
        *buildNumber = cyfx_version_build;
    }

    return CY_U3P_SUCCESS;
}

void
CyU3PSystemRegisterDriver (
        CyU3PThread *thread_p)
{
#ifdef CYU3P_PROFILE_EN
    if (glFx3DrvCount < (MAX_DRIVER_THREAD - 1))
        glFx3DrvThreads[glFx3DrvCount++] = thread_p;
#endif
}

uint32_t
CyU3PDeviceGetCpuLoad (
        void)
{
#ifdef CYU3P_PROFILE_EN
    ULONG    sa, sb, sc, sd, se, sf, sg, sh, si;
    uint32_t ts = CyU3PGetTime ();
    uint32_t load;

    CyU3PThreadSystemPerfGet (&sa, &sb, &sc, &sd, &se, &sf, &sg, &sh, &si);
    load = ((sh + si) * 30) / ts;
    load = CY_U3P_MIN (load, 100);

    return load;
#else
    return 0;
#endif
}

uint32_t
CyU3PDeviceGetThreadLoad (
        CyU3PThread *thread_p)
{
#ifdef CYU3P_PROFILE_EN
    ULONG    da, db, dc, dd, de, df, dg;
    uint32_t ts = CyU3PGetTime ();
    uint32_t load;

    CyU3PThreadPerfGet (thread_p, &da, &db, &dc, &dd, &de, &df, &dg);
    load = (da * 30) / ts;
    load = CY_U3P_MIN (load, 100);

    return load;
#else
    return 0;
#endif
}

uint32_t
CyU3PDeviceGetDriverLoad (
        void)
{
#ifdef CYU3P_PROFILE_EN
    uint32_t load = 0;
    uint8_t  i;

    for (i = 0; i < glFx3DrvCount; i++)
    {
        load += CyU3PDeviceGetThreadLoad (glFx3DrvThreads[i]);
    }
    load = CY_U3P_MIN (load, 100);

    return load;
#else
    return 0;
#endif
}

CyU3PReturnStatus_t
CyU3PSysCheckSuspendParams (
        uint16_t wakeupFlags,
        uint16_t polarity
        )
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t pol = 0;

    /* Function already executed for this context */
    if (CyU3PSysGetLPModeStatus () == CyTrue)
    {
        return status;    
    }

    glIntrEnabled = VIC->int_enable;

    /* Temporarily disable scheduling. */
    glTmpThread = CyU3PThreadIdentify ();
    if (glTmpThread == NULL)
    {
        return CY_U3P_ERROR_INVALID_CALLER;
    }

    CyU3PThreadPriorityChange (glTmpThread, 0, &glThreadPriority);   

    if ((wakeupFlags & CY_U3P_SYS_PPORT_WAKEUP_SRC) > 0)
    {
        glRegFlag = (CY_U3P_GCTL_EN_PIB_CTRL0);
    }

    /* USB as Wakeup source */
    if ((wakeupFlags & CY_U3P_SYS_USB_BUS_ACTVTY_WAKEUP_SRC) > 0)
    {
        /* Ensure that the USB has been started. */
        if (CyU3PUsbIsStarted ())
        {
            /* Check if VBUS is valid. */
            if ((GCTL->iopower & CY_U3P_VBUS) > 0)
            {
                if ((CyU3PUsbGetSpeed() == CY_U3P_HIGH_SPEED) || (CyU3PUsbGetSpeed() == CY_U3P_FULL_SPEED))
                {
                    if ((UIB->otg_ctrl & CY_U3P_UIB_DP) == 0)
                    {
                        pol |= (CY_U3P_GCTL_POL_UIB_DP);
                    }

                    if ((UIB->otg_ctrl & CY_U3P_UIB_DM) == 0)
                    {
                        pol |= (CY_U3P_GCTL_POL_UIB_DM);
                    }

                    /* Set D+ and D- as wake up sources */
                    glRegFlag |= (CY_U3P_GCTL_EN_UIB_DP | CY_U3P_GCTL_EN_UIB_DM);
                }

                if (CyU3PUsbGetSpeed() == CY_U3P_SUPER_SPEED)
                {
                    glRegFlag |= CY_U3P_GCTL_EN_UIB_SSRX;
                }
            }
            else
            {
                /* Not a valid wakeup source when VBUS is not present. */
                return CY_U3P_ERROR_BAD_ARGUMENT;
            }
        }
    }

    /* Use USB 2.0 OTGID as wake up source. */
    if ((wakeupFlags & CY_U3P_SYS_USB_OTGID_WAKEUP_SRC) > 0)
    {
        /* Check if OTG has been started in device mode. */
        if (CyU3POtgGetMode () == CY_U3P_OTG_MODE_DEVICE_ONLY)
        {
            glRegFlag |= CY_U3P_GCTL_EN_UIB_OTGID;
        }
    }

    /* Check if VBUS is a wakeup source. */
    if ((wakeupFlags & CY_U3P_SYS_USB_VBUS_WAKEUP_SRC) > 0)
    {
        if ((GCTL->iopower & CY_U3P_VBUS) > 0)
        {
            if ((polarity & CY_U3P_SYS_USB_VBUS_WAKEUP_SRC) > 0)
            {
                /* Invalid polarity */
                return CY_U3P_ERROR_BAD_ARGUMENT;
            }
        }
        else
        {
            if ((polarity & CY_U3P_SYS_USB_VBUS_WAKEUP_SRC) == 0)
            {
                /* Invalid polarity */
                return CY_U3P_ERROR_BAD_ARGUMENT;
            }

            pol |= CY_U3P_GCTL_POL_UIB_VBUS;
        }

        glRegFlag |= CY_U3P_GCTL_EN_UIB_VBUS;
    }

    /* UART as wakeup source. */
    if ((wakeupFlags & CY_U3P_SYS_UART_WAKEUP_SRC) > 0)
    {
            if ((polarity & CY_U3P_SYS_UART_WAKEUP_SRC) > 0)
            {
                pol |=  CY_U3P_GCTL_POL_UART_CTS;
            }
            glRegFlag |= CY_U3P_GCTL_EN_UART_CTS;
        }

    if (glRegFlag == 0)
    {
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }

    /* Turn off the OS timer interrupt source. */
    GCTLAON->watchdog_cs |= CY_U3P_GCTL_MODE1_MASK;

    /* Configure the wakeup sources. */
    GCTLAON->control |= CY_U3P_GCTL_WAKEUP_CPU_INT;
    GCTLAON->wakeup_en = glRegFlag;
    GCTLAON->wakeup_polarity = pol;
    CyU3PBusyWait (10);

    /* Mask all the interrupts */
    VIC->int_clear = 0xFFFFFFFF;

    /* Stop all clocks except standby clock. */
    GCTLAON->control &= ~CY_U3P_GCTL_MAIN_CLOCK_EN;
    CyU3PBusyWait (5);

    if ((GCTLAON->control & CY_U3P_GCTL_MAIN_CLOCK_EN) == 1)
    {
        /* Set the polarity to the default value. */
        GCTLAON->wakeup_polarity = CY_U3P_GCTL_WAKEUP_POLARITY_DEFAULT;
        CyU3PBusyWait (10);

        /* Clear the wakeup enable bits */
        GCTLAON->wakeup_en = CY_U3P_GCTL_WAKEUP_EN_DEFAULT;
        CyU3PBusyWait (10);

        /* Restore the interrupts that were masked earlier on. */
        VIC->int_enable = glIntrEnabled;
        /* Restore the calling thread priority. */
        CyU3PThreadPriorityChange (glTmpThread, glThreadPriority, &glThreadPriority);

        /* Abort the suspend procedure as there are still wakeup events set in
         * the WAKEUP_EVENT register that needs to handled. */
        return CY_U3P_ERROR_ABORTED;
    }

    if (status == CY_U3P_SUCCESS)
    {
        CyU3PSysSetLPModeStatus (CyTrue);
    }

    return status;
}

void
CyU3PSysSendEnterSuspendStatus (
        void
        )
{
    VIC->int_enable = (1 << CY_U3P_VIC_PIB_CORE_VECTOR);

    CyU3PMboxWait ();
    VIC->int_clear = (1 << CY_U3P_VIC_PIB_CORE_VECTOR);

    CyU3PBusyWait (100);
}

/* This function moves the device into suspend mode. */
static CyU3PReturnStatus_t
MySysEnterSuspendMode (
        uint16_t wakeupFlags,
        uint16_t polarity,
        uint16_t *wakeupSource
        )
{
    register uint32_t armReg = 0;
    CyBool_t uibClkOn = CyFalse;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
       
    status = CyU3PSysCheckSuspendParams (wakeupFlags, polarity);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }
    
    if (CyU3PSysGetLPModeStatus () == CyFalse)
    {
        glIntrEnabled = VIC->int_enable;
    }
    else
    {
        CyU3PSysSetLPModeStatus (CyFalse);
    }

    /* Mask the IRQ interrupt on ARM so that the GCTL interrupt is never handled. */
    __disable_irq();

    if (CyU3PUsbGetSpeed () != CY_U3P_SUPER_SPEED)
    {
        /* Disable the USB clock. */
        if ((GCTL->uib_core_clk & CY_U3P_GCTL_UIBCLK_CLK_EN) != 0)
            uibClkOn = CyTrue;
        GCTL->uib_core_clk &= ~CY_U3P_GCTL_UIBCLK_CLK_EN;
    }

    /* Clear any pending wakeup events and ensure that the wakeup enable mask is
       set correctly. */
    GCTLAON->wakeup_event = GCTLAON->wakeup_event;
    if (GCTLAON->wakeup_en != glRegFlag)
	GCTLAON->wakeup_en = glRegFlag;

    /* Enable the GCTL core interrupt. */    
    VIC->int_enable = (1 << CY_U3P_VIC_GCTL_CORE_VECTOR);

    /* Clean the D-Cache */ 
    CyU3PSysCleanDCache ();

    /* Switch the ARM processor into a low power state until an interrupt occurs. */
#ifdef CY_USE_ARMCC
    __asm {
        MCR p15, 0, armReg, c7, c0, 4;
    }
#else
    __asm__ __volatile__
        (
         "MCR p15, 0, %0, c7, c0, 4\n\t"
         : "+r" (armReg)
         :
         :
        );
#endif

    CyU3PBusyWait (5);

    glRegFlag = (glRegFlag & GCTLAON->wakeup_event);
    *wakeupSource = 0;

    if (glRegFlag & CY_U3P_GCTL_EN_PIB_CTRL0)
    {
        *wakeupSource |= CY_U3P_SYS_PPORT_WAKEUP_SRC;
    }
    if (glRegFlag & CY_U3P_GCTL_EN_UART_CTS)
    {
        *wakeupSource |= CY_U3P_SYS_UART_WAKEUP_SRC;
    }
    if (glRegFlag & (CY_U3P_GCTL_EN_UIB_DP | CY_U3P_GCTL_EN_UIB_DM | CY_U3P_GCTL_EN_UIB_SSRX))
    {
        *wakeupSource |= CY_U3P_SYS_USB_BUS_ACTVTY_WAKEUP_SRC;
    }
    if (glRegFlag & CY_U3P_GCTL_EN_UIB_OTGID)
    {
        *wakeupSource |= CY_U3P_SYS_USB_OTGID_WAKEUP_SRC;
    }
    if (glRegFlag & CY_U3P_GCTL_EN_UIB_VBUS)
    {
        *wakeupSource |= CY_U3P_SYS_USB_VBUS_WAKEUP_SRC;
    }

    /* Re-enable the OS timer interrupt. */
    CyU3POsTimerInit (glOsTimerInterval);

    if (CyU3PUsbGetSpeed () != CY_U3P_SUPER_SPEED)
    {
        if (uibClkOn)
            GCTL->uib_core_clk |= CY_U3P_GCTL_UIBCLK_CLK_EN;
    }

    /* Re-enable IRQ handling so that the GCTL CORE interupt
       is handled and the wakeup sources removed. */
#ifdef CY_USE_ARMCC
    __enable_irq();
#else
     __asm__ __volatile__
         (
          "stmdb sp!, {r0}\n\t"                  /* Save R0 value on the stack. */
          "mrs   r0, CPSR\n\t"                   /* Read current CPSR value. */
          "bic   r0, r0, #0xC0\n\t"              /* Enable IRQ and FIQ. */
          "msr   CPSR_c, r0\n\t"                 /* Write back to disable interrupts. */
          "ldmia sp!, {r0}\n\t"                  /* Restore R0 register content. */
         );
#endif

    /* Restore the calling thread priority. */
    CyU3PThreadPriorityChange (glTmpThread, glThreadPriority, &glThreadPriority);

    /* Restore the interrupts that were masked prior to entering the suspend mode. */
    VIC->int_enable = glIntrEnabled;

    return CY_U3P_SUCCESS;
}

/* This function moves the Fx3 device into suspend mode. */
CyU3PReturnStatus_t
CyU3PSysEnterSuspendMode (
        uint16_t wakeupFlags,
        uint16_t polarity,
        uint16_t *wakeupSource
        )
{
    if (((wakeupFlags & 0x1F) == 0) || (wakeupSource == NULL))
    {
        /* Wakeup sources are not valid. */
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }

    return MySysEnterSuspendMode  (wakeupFlags & 0x1F, polarity, wakeupSource);
}

uint32_t *
CyU3PSysGetRegBkpAddr (
        void)
{
    return (glDevBootState.regBkpAddr);
}

void
CyU3PSysCheckBootState (
        void)
{
    uint32_t  addr;
    uint8_t  *buff_p;

    glDevBootState.regBkpAddr = 0;

    if (((GCTLAON->control & CY_U3P_GCTL_FREEZE_IO) != 0) && (glDevBootState.signature == FX3_STANDBY_SIG) &&
            (glDevBootState.isWarmBoot == CyTrue))
    {
        glDevBootState.isWarmBoot = CyFalse;

        /* Restore the I-TCM content when coming out of standby mode. */
        addr   = *((uint32_t *)0x40000008);
        buff_p = (uint8_t *)addr;
        CyU3PMemCopy ((uint8_t *)CY_U3P_ITCM_BASE, buff_p, CY_U3P_ITCM_SIZE);

        /* Check whether we should enable the GPIO block before unfreezing the IOs. */
        if (*((uint32_t *)0x4000000C) == 0x55555555)
        {
            glDevBootState.regBkpAddr = (uint32_t *)(addr + CY_U3P_ITCM_SIZE);
        }
    }
}

extern CyBool_t
CyFx3IsUsbActive (
        void);
extern CyBool_t
CyU3PAreLppsOff (
        uint32_t retainGpioState);
extern CyU3PLppInterruptHandler glLppInterruptHandler[];

CyU3PReturnStatus_t
CyU3PSysCheckStandbyParam (
        uint16_t wakeupFlags,
        uint16_t polarity,
        uint8_t  *bkp_buff_p)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t pol = 0;

    if (CyU3PSysGetLPModeStatus () == CyTrue)
    {
        return status;
    }

    glIntrEnabled = VIC->int_enable;

    /* If the USB block or any of the LPP blocks (except GPIO) are active; fail the attempt to switch to standby
       mode. */
    if ((CyFx3IsUsbActive ()) || (!CyU3PAreLppsOff (CyTrue)))
        return CY_U3P_ERROR_INVALID_SEQUENCE;    

    /* Make sure a valid buffer location is provided for saving the ITCM content. */
    if (((uint32_t)bkp_buff_p < 0x40000000) || ((uint32_t)bkp_buff_p >= 0x40080000))
        return CY_U3P_ERROR_BAD_ARGUMENT;

    if ((wakeupFlags & CY_U3P_SYS_PPORT_WAKEUP_SRC) > 0)
    {
        glRegFlag = (CY_U3P_GCTL_EN_PIB_CTRL0);
    }

    /* Check if VBUS is a wakeup source. */
    if ((wakeupFlags & CY_U3P_SYS_USB_VBUS_WAKEUP_SRC) > 0)
    {
        if ((GCTL->iopower & CY_U3P_VBUS) > 0)
        {
            if ((polarity & CY_U3P_SYS_USB_VBUS_WAKEUP_SRC) > 0)
            {
                /* Invalid polarity */
                return CY_U3P_ERROR_BAD_ARGUMENT;
            }
        }
        else
        {
            if ((polarity & CY_U3P_SYS_USB_VBUS_WAKEUP_SRC) == 0)
            {
                /* Invalid polarity */
                return CY_U3P_ERROR_BAD_ARGUMENT;
            }

            pol |= CY_U3P_GCTL_POL_UIB_VBUS;
        }

        glRegFlag |= CY_U3P_GCTL_EN_UIB_VBUS;
    }

    /* ToDo: CDT 168827 Update */
    /* UART as wakeup source. */
    if ((wakeupFlags & CY_U3P_SYS_UART_WAKEUP_SRC) > 0)
    {
        if ((polarity & CY_U3P_SYS_UART_WAKEUP_SRC) > 0)
        {
            pol |=  CY_U3P_GCTL_POL_UART_CTS;
        }
        glRegFlag |= CY_U3P_GCTL_EN_UART_CTS;
    }

    if (glRegFlag == 0)
    {
        /* No valid wakeup source specified. */
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }

    /* Temporarily disable scheduling. */
    glTmpThread = CyU3PThreadIdentify ();
    if (glTmpThread == NULL)
    {
        return CY_U3P_ERROR_INVALID_CALLER;
    }
    CyU3PThreadPriorityChange (glTmpThread, 0, &glThreadPriority);

    /* Configure the wakeup sources. */
    GCTLAON->control |= CY_U3P_GCTL_WAKEUP_CPU_INT;
    GCTLAON->wakeup_en       = glRegFlag;
    GCTLAON->wakeup_polarity = pol;
    CyU3PBusyWait (10);   

    if (status == CY_U3P_SUCCESS)
    {
        CyU3PSysSetLPModeStatus (CyTrue);
    }

    return status;
}

/* This function moves the FX3 device into standby mode. */
CyU3PReturnStatus_t
CyU3PSysEnterStandbyMode (
        uint16_t  wakeupFlags,
        uint16_t  polarity,
        uint8_t  *bkp_buff_p)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t     temp1 = 0;
    uint32_t    *reg_buff;

    status = CyU3PSysCheckStandbyParam (wakeupFlags, polarity, bkp_buff_p);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    if (CyU3PSysGetLPModeStatus () == CyFalse)
    {
        glIntrEnabled = VIC->int_enable;
    }
    else
    {
        CyU3PSysSetLPModeStatus (CyFalse);
    }

    /* Mask all the interrupts */
    VIC->int_clear = 0xFFFFFFFF;
    CyU3PBusyWait (100);

    /* Mask the IRQ interrupt on ARM so that the GCTL interrupt is never handled. */
    __disable_irq();

    /* Clear any pending wakeup events and ensure that the wakeup enable mask is
       set correctly. */
    GCTLAON->wakeup_event = GCTLAON->wakeup_event;
    if (GCTLAON->wakeup_en != glRegFlag)
    {
        GCTLAON->wakeup_en = glRegFlag;
    }

    /* Disable the Watchdog Timers */
    temp1 = GCTLAON->watchdog_cs;
    CyU3PBusyWait (100);
    temp1 |= CY_U3P_GCTL_MODE0_MASK | CY_U3P_GCTL_MODE1_MASK;
    GCTLAON->watchdog_cs = temp1;
    CyU3PBusyWait (100);

    GCTLAON->control &= ~(CY_U3P_GCTL_POR | CY_U3P_GCTL_SW_RESET | CY_U3P_GCTL_WDT_RESET |
            CY_U3P_GCTL_WAKEUP_PWR | CY_U3P_GCTL_WAKEUP_CLK);
    CyU3PBusyWait (10);

    /* Power down the VCore Power Domain. */
    GCTLAON->control &= ~(CY_U3P_GCTL_MAIN_POWER_EN);
    CyU3PBusyWait (10);
    if ((GCTLAON->control & CY_U3P_GCTL_MAIN_POWER_EN) != 0)
    {
        /* Wake up events are set. Can't enter standby mode. Return to normal operating sequence. */
        CyU3POsTimerInit (glOsTimerInterval);
        VIC->int_enable = glIntrEnabled;
#ifdef CY_USE_ARMCC
        __enable_irq();
#else
        __asm__ __volatile__
            (
             "stmdb sp!, {r0}\n\t"                  /* Save R0 value on the stack. */
             "mrs   r0, CPSR\n\t"                   /* Read current CPSR value. */
             "bic   r0, r0, #0xC0\n\t"              /* Enable IRQ and FIQ. */
             "msr   CPSR_c, r0\n\t"                 /* Write back to disable interrupts. */
             "ldmia sp!, {r0}\n\t"                  /* Restore R0 register content. */
            );
#endif

        /* Restore the calling thread priority. */
        CyU3PThreadPriorityChange (glTmpThread, glThreadPriority, &glThreadPriority);
        goto errorReturn;

    }

    /* Going into standby state now. */
    glDevBootState.signature  = FX3_STANDBY_SIG;
    glDevBootState.isWarmBoot = CyTrue;

    /* Make sure all Heap memory is freed up, and then backup the ITCM content. */
    CyU3PFreeHeaps ();
    CyU3PMemCopy (bkp_buff_p, (uint8_t *)CY_U3P_ITCM_BASE, CY_U3P_ITCM_SIZE);
    *((uint32_t *)0x40000000) = (uint32_t)&CyU3PFirmwareEntry;
    *((uint32_t *)0x40000008) = (uint32_t)bkp_buff_p;

    /* If the GPIO block is still on, back up all GPIO register values. */
    if (((GCTL->gpio_fast_clk & CY_U3P_GCTL_GPIOFCLK_CLK_EN) != 0) &&
            ((GPIO->lpp_gpio_power & CY_U3P_LPP_GPIO_ACTIVE) != 0))
    {
        *((uint32_t *)0x4000000C) = 0x55555555;
        reg_buff = (uint32_t *)(bkp_buff_p + CY_U3P_ITCM_SIZE);

        reg_buff[0] = GCTL->gpio_fast_clk;
        reg_buff[1] = GCTL->gpio_slow_clk;
        reg_buff[2] = GCTL->gpio_simple0;
        reg_buff[3] = GCTL->gpio_simple1;
        reg_buff[4] = GCTL->iomatrix;
        reg_buff[5] = ((uint32_t *)glLppInterruptHandler)[CY_U3P_LPP_GPIO];
        for (temp1 = 0; temp1 < 61; temp1++)
        {
            reg_buff[temp1 + 6] = GPIO->lpp_gpio_simple[temp1];
        }
    }
    else
    {
        *((uint32_t *)0x4000000C) = 0;
    }
 
    /* Clean the D-Cache and disable it */ 
    CyU3PSysCleanDCache ();
    CyU3PSysDisableDCache ();
    CyU3PSysDisableICache ();

    /* Freeze the IOs */
    GCTLAON->freeze = 0x00;
    GCTLAON->control |= CY_U3P_GCTL_FREEZE_IO;

    /* Enable the GCTL core interrupt. */    
    VIC->int_enable = (1 << CY_U3P_VIC_GCTL_CORE_VECTOR);

    /* Set the Warm boot bit */
    GCTLAON->control |= CY_U3P_GCTL_WARM_BOOT;

    /* Place the ARM core in a wait for interrupt state to initiate standby state. */
#ifdef CY_USE_ARMCC
    __asm
    {
        MCR p15, 0, temp1, c7, c0, 4
    }
#else
    __asm__ __volatile__
        (
         "MCR p15, 0, %0, c7, c0, 4\n\t"
         : "+r" (temp1)
         :
         :
        );
#endif

    for (;;)
        CyU3PBusyWait (5);

errorReturn:
    return CY_U3P_ERROR_STANDBY_FAILED;
}

/*
 * System Thread Entry function 
 */
void
CyU3PSysThreadEntry (
        uint32_t input)
{
    uint32_t flag, mask;
    unsigned int status1;

    /* Wait for the initialzation completion event from all modules */
    mask = (0x10 << CY_U3P_PIB_MODULE_ID);
    mask |= (0x10 << CY_U3P_STOR_MODULE_ID);
    CyU3PEventGet (&glSysEvent, mask, CYU3P_EVENT_AND_CLEAR, &flag, CYU3P_WAIT_FOREVER);
    
    /* The priority is set to the highest to make sure that 
     * the function call completes */
    CyU3PThreadPriorityChange (&glSysThread, 0, &status1);

    CyFxApplicationDefine ();
    CyU3PThreadPriorityChange (&glSysThread, CY_U3P_SYS_THREAD_PRIORITY, &status1);
    
    while (1)
    {
        /* This thread has nothing to do from now. */
        CyU3PThreadSleep (10000);
    }
}

/*
 * The primary Application define function
 */
void
CyU3PApplicationDefine (
        void)
{
    void *ptr;

    /* Global variable re-initialization. */
    glDevBootState.isWarmBoot = CyFalse;
    glDevBootState.signature  = FX3_ACTIVE_SIG;

    /* Start the DMA driver first. */
    CyU3PMemInit ();
    CyU3PDmaApplicationDefine ();

    CyU3PEventCreate (&glSysEvent);

    ptr = CyU3PMemAlloc (CY_U3P_SYS_THREAD_STACK);
    CyU3PThreadCreate (&glSysThread, "02_SYSTEM_THREAD", CyU3PSysThreadEntry, 0,
            ptr, CY_U3P_SYS_THREAD_STACK, CY_U3P_SYS_THREAD_PRIORITY, CY_U3P_SYS_THREAD_PRIORITY, 
            CYU3P_NO_TIME_SLICE, CYU3P_AUTO_START);
    CyU3PSystemRegisterDriver (&glSysThread);

    /* Start all of the other drivers. */
    CyU3PSibApplicationDefine ();
    CyU3PDebugApplicationDefine ();
    CyU3PPibApplicationDefine ();
    CyU3PLppApplicationDefine ();
    CyU3PUibApplicationDefine ();
}

/*
 * Informs system thread about completion of corresponding module initialization
 */
void
CyU3PSysModuleInitCompleteEvt (
       CyU3PModuleId_t module)
{
    /* Use the 5th to 12ve event flags for indiacting initialization complete event */
    CyU3PEventSet (&glSysEvent, 0x10 << module, CYU3P_EVENT_OR);
}

extern void
CyU3PGctlCoreIntHandler (
        void) __attribute__ ((section ("CYU3P_ITCM_SECTION")));
extern void
CyU3PSwiIntHandler (
        void) __attribute__ ((section ("CYU3P_ITCM_SECTION")));

void
CyU3PGctlCoreIntHandler (
        void)
{
    /* USB 2.0 LPM-L1 workaround. */
    if (UIB->ohci_rh_port_status & CY_U3P_UIB_RHP_PORT_RESUME_FW)
    {
        if (GCTLAON->wakeup_event & CY_U3P_GCTL_EV_UIB_DP)
        {
            UIB->phy_chirp            = 0x00000000; /* Clear the CY_U3P_UIB_OVERRIDE_FSM bit. */
            UIB->dev_pwr_cs          &= ~(CY_U3P_UIB_DEV_SUSPEND | CY_U3P_UIB_SIGRSUME);
            UIB->ohci_rh_port_status &= ~CY_U3P_UIB_RHP_PORT_RESUME_FW;
        }
    }

    /* Clear the wakeup enable bits */
    GCTLAON->wakeup_en = CY_U3P_GCTL_WAKEUP_EN_DEFAULT;
    CyU3PBusyWait (10);

    /* Clear the wakeup event(s) responsible for the wakeup. */
    GCTLAON->wakeup_event = 0x3FFF;
    CyU3PBusyWait (10);

    /* Clear the GCTL core interrupt for now. */
    VIC->int_clear = (1 << CY_U3P_VIC_GCTL_CORE_VECTOR);
}

void
CyU3PSwiIntHandler (
        void)
{
    /* Mask the SWI interrupt */
    VIC->int_clear = CY_U3P_VIC_SWI_VECTOR;

    /* Clear the SWI interrupt */
    CyFx3DevClearSwInterrupt ();
}

/* [] */

