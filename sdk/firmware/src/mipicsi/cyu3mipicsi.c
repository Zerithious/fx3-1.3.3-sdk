/*
 ## Cypress CX3 MIPI-CSI Driver Source (cyu3mipicsi.c)
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

#include <cyu3system.h>
#include <cyu3types.h>
#include <cyu3utils.h>
#include <cyu3regs.h>
#include <cyu3error.h>
#include <cyu3protocol.h>
#include <cyu3vic.h>
#include <cyu3gpio.h>
#include <cyu3pib.h>
#include <cyu3i2c.h>
#include <cyu3mipicsi.h>
#include <cyfx3_api.h>

/* Read and Write I2C Device addresses for the MIPI-CSI Block */
#define CY_U3P_MIPICSI_READ_ADDR    (0x0F)
#define CY_U3P_MIPICSI_WRITE_ADDR   (0x0E)

/* MIPI CSI Block Register Addresses */
#define CY_U3P_MIPICSI_REG_ID       (0x00)
#define CY_U3P_MIPICSI_REG_SYSCTL   (0x02)
#define CY_U3P_MIPICSI_REG_CONFCTL  (0x04)
#define CY_U3P_MIPICSI_REG_FIFOCTL  (0x06)
#define CY_U3P_MIPICSI_REG_DATAFMT  (0x08)
#define CY_U3P_MIPICSI_REG_MCLKCTL  (0x0C)
#define CY_U3P_MIPICSI_REG_GPIOEN   (0x0E)
#define CY_U3P_MIPICSI_REG_GPIODIR  (0x10)
#define CY_U3P_MIPICSI_REG_GPIOIN   (0x12)
#define CY_U3P_MIPICSI_REG_GPIOOUT  (0x14)
#define CY_U3P_MIPICSI_REG_PLLCTL0  (0x16)
#define CY_U3P_MIPICSI_REG_PLLCTL1  (0x18)
#define CY_U3P_MIPICSI_REG_CLKCTRL  (0x20)
#define CY_U3P_MIPICSI_REG_WORDCNT  (0x22)
#define CY_U3P_MIPICSI_REG_PHYCLKCTL   (0x56)
#define CY_U3P_MIPICSI_REG_PHYDAT0CTL  (0x58)
#define CY_U3P_MIPICSI_REG_PHYDAT1CTL  (0x5A)
#define CY_U3P_MIPICSI_REG_PHYDAT2CTL  (0x5C)
#define CY_U3P_MIPICSI_REG_PHYDAT3CTL  (0x5E)
#define CY_U3P_MIPICSI_REG_PHYTIMDLY   (0x60)
#define CY_U3P_MIPICSI_REG_PHY_STAT    (0x62)
#define CY_U3P_MIPICSI_REG_CSI_STAT    (0x64)
#define CY_U3P_MIPICSI_REG_CSI_ERR_EN  (0x66)
#define CY_U3P_MIPICSI_REG_MDLSYN_ERR  (0x68)
#define CY_U3P_MIPICSI_REG_CSI_DID     (0x6A)
#define CY_U3P_MIPICSI_REG_CSI_DID_ERR (0x6C)
#define CY_U3P_MIPICSI_REG_CSI_PKTLEN  (0x6E)
#define CY_U3P_MIPICSI_REG_CSIRX_DPCTL (0x70)
#define CY_U3P_MIPICSI_REG_FRM_ERR_CNT (0x80)
#define CY_U3P_MIPICSI_REG_CRC_ERR_CNT (0x82)
#define CY_U3P_MIPICSI_REG_COR_ERR_CNT (0x84)
#define CY_U3P_MIPICSI_REG_HDR_ERR_CNT (0x86)
#define CY_U3P_MIPICSI_REG_EID_ERR_CNT (0x88)
#define CY_U3P_MIPICSI_REG_CTL_ERR_CNT (0x8A)
#define CY_U3P_MIPICSI_REG_SOT_ERR_CNT (0x8C)
#define CY_U3P_MIPICSI_REG_SYN_ERR_CNT (0x8E)

#define CY_U3P_MIPICSI_REG_MDL_ERR_CNT (0x90)
#define CY_U3P_MIPICSI_REG_FIFO_STAT   (0xF8)

#define CY_U3P_MIPICSI_BLK_ID       (0x00)
#define CY_U3P_MIPICSI_BLK_REV      (0x44)
#define CY_U3P_MIPICSI_SET_RESET    (0x01)
#define CY_U3P_MIPICSI_CLR_RESET    (0x00)
#define CY_U3P_MIPICSI_BLOCK_SLEEP  (0x02)
#define CY_U3P_MIPICSI_BLOCK_WAKEUP (0x00)

#define CY_U3P_MIPICSI_RESET_GPIO      (27)

#define CY_U3P_MIPICSI_PLL_FBD_MAX      (0x1FF)
#define CY_U3P_MIPICSI_PLL_PRD_MAX      (0x0F)
#define CY_U3P_MIPICSI_FIFO_DELAY_MAX   (0x1FF)
#define CY_U3P_MIPICSI_HRES_MAX         (0xFFF8)
#define CY_U3P_MIPICSI_HRES_MIN         (0x0007)

#define CY_U3P_MIPICSI_CONFCTL_LOW_DEFAULT     (0x40)
#define CY_U3P_MIPICSI_PLL_LBWS_DEFAULT        (0x02)
#define CY_U3P_MIPICSI_PLLCTL1_LOW_INITIAL     (0x03)
#define CY_U3P_MIPICSI_PLLCTL1_LOW_DEFAULT     (0x13)

#define CY_U3P_MIPICSI_DATAFMT_MASK(x)         ((x) & 0x0F)
#define CY_U3P_MIPICSI_DATAMOD_MASK(x)         (((x) & 0xF0)>>(0x04))

#define CY_U3P_MIPICSI_THS_PHY_DELAY_MASK(t,s)    (((t & 0x01) << 7) | (s & 0x7F))

#define CY_U3P_MIPICSI_SYNERR_MASK              (0x0001)
#define CY_U3P_MIPICSI_SOTERR_MASK              (0x0002)
#define CY_U3P_MIPICSI_CTLERR_MASK              (0x0004)
#define CY_U3P_MIPICSI_EIDERR_MASK              (0x0008)
#define CY_U3P_MIPICSI_HDRERR_MASK              (0x0010)
#define CY_U3P_MIPICSI_CORERR_MASK              (0x0020)
#define CY_U3P_MIPICSI_CRCERR_MASK              (0x0040)
#define CY_U3P_MIPICSI_FRMERR_MASK              (0x0080)
#define CY_U3P_MIPICSI_MDLERR_MASK              (0x0100)



/* Check to verify CX3 MIPI block has been found and device is not in Reset*/
CyBool_t glMipiBlockInitialized = CyFalse;
/* Check to get state of Block - Sleep or Active*/
CyBool_t glIsMipiBlockActive = CyFalse;

/* Number of Mipi Lanes on the Device*/
uint8_t glPartMipiLanes = 0;
uint8_t glUserDataTypeEnable = 1;

/* ---------------------------------------------------------------------------- */
/* --------------------- Internal Only Functions ------------------------------ */
/* ---------------------------------------------------------------------------- */

/* Read a configuration Register on the MIPI interface block over I2C*/
static CyU3PReturnStatus_t
CyU3PMipicsiRegisterRead (
        uint16_t regAddr,
        uint8_t  retryCount,
        uint8_t *rdData)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;
    uint8_t i;

    for (i = 0; i < retryCount; i++)
    {

        preamble.buffer[0] = CY_U3P_MIPICSI_WRITE_ADDR;
        preamble.buffer[1] = CY_U3P_GET_MSB(regAddr);
        preamble.buffer[2] = CY_U3P_GET_LSB(regAddr);
        preamble.buffer[3] = CY_U3P_MIPICSI_READ_ADDR; /* Slave address: Read operation */
        preamble.length = 4;
        preamble.ctrlMask = 0x0004;

        status = CyU3PI2cReceiveBytes (&preamble, rdData, 2, 0);
        if (status != CY_U3P_SUCCESS)
            CyU3PThreadSleep (1);
        else
            break;
    }

    return status;
}

/* Write a configuration register on the MIPI block over I2C*/
static CyU3PReturnStatus_t
CyU3PMipicsiRegisterWrite (
        uint16_t regAddr,
        uint8_t  retryCount,
        uint8_t *data)
{
    uint8_t i;
    CyU3PReturnStatus_t status=CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;

    for(i = 0; i <= retryCount ; i++)
    {
        preamble.buffer[0] = CY_U3P_MIPICSI_WRITE_ADDR; /* Slave address: Write operation */
        preamble.buffer[1] = CY_U3P_GET_MSB(regAddr);
        preamble.buffer[2] = CY_U3P_GET_LSB(regAddr);
        preamble.length = 3;
        preamble.ctrlMask = 0x0000;

        status = CyU3PI2cTransmitBytes (&preamble, data, 2, 0);
        if (status != CY_U3P_SUCCESS)
            CyU3PThreadSleep (1);
        else
            break;
    }
    return status;
}

/* Verify the Block ID and Revision ID registers on the MIPI block*/
static CyU3PReturnStatus_t
CyU3PMipicsiVerifyBlockId (
        void)
{
    CyU3PReturnStatus_t status;
    uint8_t idBuffer[2];

    status = CyU3PMipicsiRegisterRead( CY_U3P_MIPICSI_REG_ID, 2, idBuffer);
    if(status != CY_U3P_SUCCESS)
    {
        return CY_U3P_ERROR_NOT_SUPPORTED;
    }
    if ((idBuffer[1] != CY_U3P_MIPICSI_BLK_ID) && (idBuffer[0] != CY_U3P_MIPICSI_BLK_REV))
    {
        return CY_U3P_ERROR_NOT_SUPPORTED;
    }
    return status;
}

/* Enable the sensor reset GPIOs on the MIPI block*/
static CyU3PReturnStatus_t
CyU3PMipicsiSetupSensorResetGpio (
        void)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint8_t rwBuffer[2];

    /* Set GPIOs to Output */
    rwBuffer[0] = rwBuffer[1] = 0x00;

    /* Read the GPIODir Register */
    status = CyU3PMipicsiRegisterRead (CY_U3P_MIPICSI_REG_GPIODIR, 2, rwBuffer);
    if(status != CY_U3P_SUCCESS)
        return status;
    /* Set the XRES and XShuDown lines to Output => 0*/
    rwBuffer[1] &= (~ (CY_U3P_CSI_IO_XRES | CY_U3P_CSI_IO_XSHUTDOWN));

    status = CyU3PMipicsiRegisterWrite (CY_U3P_MIPICSI_REG_GPIODIR, 2, rwBuffer);

    return status;
}

/* Software reset on the MIPI block. setting noClearReset leaves the block in a reset state*/
static CyU3PReturnStatus_t
CyU3PMipicsiSoftReset (
        CyBool_t noClearReset)
{
    CyU3PReturnStatus_t status;

    uint8_t buffer[2];

    /* Put the block into Reset */
    buffer[0] = 0x00;
    buffer[1] = CY_U3P_MIPICSI_SET_RESET;
    status = CyU3PMipicsiRegisterWrite (CY_U3P_MIPICSI_REG_SYSCTL, 2, buffer);
    if(status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint(4,"\rMipi_soft_Reset Set failed\r\n");
        return status;
    }

    if (noClearReset)
        return status;

    CyU3PThreadSleep(5);

    /* Clear Reset */
    buffer[0] = 0x00;
    buffer[1] = CY_U3P_MIPICSI_CLR_RESET;
    status = CyU3PMipicsiRegisterWrite (CY_U3P_MIPICSI_REG_SYSCTL, 2, buffer);
    if(status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint(4,"\rMipi_soft_Reset Clear failed\r\n");
        return status;
    }
    return status;
}

/* Hard reset the MIPI block*/
static CyU3PReturnStatus_t
CyU3PMipicsiHardReset (
        CyBool_t noClearReset)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    status = CyU3PGpioSetValue (CY_U3P_MIPICSI_RESET_GPIO, CyFalse);
    if ((status != CY_U3P_SUCCESS) || (noClearReset != CyFalse))
    {
        return status;
    }
    CyU3PThreadSleep(5);

    status = CyU3PGpioSetValue (CY_U3P_MIPICSI_RESET_GPIO, CyTrue);

    CyU3PThreadSleep(5);

    /* Configure the IS reset and shutdown lines as Outputs */
    status = CyU3PMipicsiSetupSensorResetGpio();

    return status;
}


/* Setup the Reset GPIO to enable Hard Resets*/
static CyU3PReturnStatus_t
CyU3PMipicsiSetupBlockResetGpio (
        void)
{
    CyU3PGpioSimpleConfig_t gpioConfig;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    status = CyU3PDeviceGpioOverride (CY_U3P_MIPICSI_RESET_GPIO, CyTrue);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }
    gpioConfig.outValue = CyFalse;
    gpioConfig.driveLowEn = CyTrue;
    gpioConfig.driveHighEn = CyTrue;
    gpioConfig.inputEn = CyFalse;
    gpioConfig.intrMode = CY_U3P_GPIO_NO_INTR;
    status = CyU3PGpioSetSimpleConfig(CY_U3P_MIPICSI_RESET_GPIO, &gpioConfig);
    return status;
}



/* Enable logging of errors on the device */
static CyU3PReturnStatus_t
CyU3PMipicsiEnableMipiErrors (
        void)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    /* Set GPIOs to Output */

    uint8_t wrBuffer[2];

    wrBuffer[0] = 0x01;
    wrBuffer[1] = 0xFF;

    /* Write to the ERR_ENABLE the GPIODir Register */

    status = CyU3PMipicsiRegisterWrite (CY_U3P_MIPICSI_REG_CSI_ERR_EN, 2, wrBuffer);
    return status;
}

/* Override routine for UDT Enable. Only for prototyping for Intel. Not for public release on SDK 1.0 */
void
CyU3PMipicsiSetFollowCSIDataType (
        CyBool_t csiDataType)
{
    if(csiDataType)
        glUserDataTypeEnable = 0;
    else
        glUserDataTypeEnable = 1;
}


/* ---------------------------------------------------------------------------- */
/* ----------------------------- Public APIs ---------------------------------- */
/* ---------------------------------------------------------------------------- */

/* Initiailize the MIPI-CSI block. The GPIO and I2C blocks need to have been initiailized prior to calling this function */
CyU3PReturnStatus_t
CyU3PMipicsiInit (
        void)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    if(glMipiBlockInitialized)
        return CY_U3P_SUCCESS;

    /* Check if the device has Mipi CSI Support CX2/CX3 and set a flag */
    if (!(CyFx3DevIsMipicsiSupported()))
    {
        return CY_U3P_ERROR_NOT_SUPPORTED;
    }


    /* Setup number of Mipi lanes on the part based on PartInfo.*/
    glPartMipiLanes = CyFx3DevGetMipiLaneCount ();

    /* Code to Initialize the Reset line GPIO pin goes here. */
    status = CyU3PMipicsiSetupBlockResetGpio ();
    if (status != CY_U3P_SUCCESS)
        return status;

    /* Hard Reset the Mipi CSI Block by  calling MipiReset with the param for hard reset*/
    /*    Also reads and validates the ID register on the MIPI block  and sets the CXRES
     *    andCXSHUTDOWN lines to DRIVE LOW*/
    status = CyU3PMipicsiReset (CY_U3P_CSI_HARD_RST);
    if (status != CY_U3P_SUCCESS)
        return status;



    /* Enable MIPI block errors*/
    status = CyU3PMipicsiEnableMipiErrors ();
    if (status != CY_U3P_SUCCESS)
        return status;

    glIsMipiBlockActive = CyTrue;
    status = CyU3PMipicsiSleep();

    return status;

}

CyU3PReturnStatus_t
CyU3PMipicsiDeInit (
        void)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    if(!glMipiBlockInitialized)
        return CY_U3P_SUCCESS;

    /* Check if the device has Mipi CSI Support CX2/CX3 and set a flag */
    if (!(CyFx3DevIsMipicsiSupported()))
    {
        return CY_U3P_ERROR_NOT_SUPPORTED;
    }

    /* Put the bridge into reset and leave it in reset state */

    status = CyU3PMipicsiHardReset(CyTrue);
    if (status != CY_U3P_SUCCESS)
        return status;

    glMipiBlockInitialized = glIsMipiBlockActive = CyFalse; /* Reset the initialization flags before Reset device*/
    return status;

}

/* Set Interface Configuration Parameters*/
CyU3PReturnStatus_t
CyU3PMipicsiSetIntfParams  (
        CyU3PMipicsiCfg_t *csiCfg,    /* Configuration Data */
        CyBool_t wakeOnConfigure      /* Start the block clocks immediately after configuring */
        )
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint8_t pdfMode; /* Parallel Data Format Mode Value*/
    uint8_t pdfValue; /* Parallel Data Format. */
    uint32_t wordCount; /* Word count*/
    uint8_t wrBuffer[2];

    /* Check if MIPI clock is initialized*/

    if(!glMipiBlockInitialized)
        return CY_U3P_ERROR_NOT_STARTED;
    if (csiCfg == NULL)
    {
        return CY_U3P_ERROR_NULL_POINTER;
    }

    /* Data Sanity Checks*/
    if((csiCfg->pllPrd > CY_U3P_MIPICSI_PLL_PRD_MAX) || (csiCfg->pllFbd > CY_U3P_MIPICSI_PLL_FBD_MAX)
            || (csiCfg->fifoDelay > CY_U3P_MIPICSI_FIFO_DELAY_MAX) )
        return CY_U3P_ERROR_BAD_ARGUMENT;

    if(csiCfg->pllFrs >  CY_U3P_CSI_PLL_FRS_63_125M)
        return CY_U3P_ERROR_BAD_ARGUMENT;

    if (csiCfg->csiRxClkDiv >= CY_U3P_CSI_PLL_CLK_DIV_INVALID)
        return CY_U3P_ERROR_BAD_ARGUMENT;

    if (csiCfg->mClkRefDiv  >= CY_U3P_CSI_PLL_CLK_DIV_INVALID)
        return CY_U3P_ERROR_BAD_ARGUMENT;

    if (csiCfg->parClkDiv >= CY_U3P_CSI_PLL_CLK_DIV_INVALID)
        return CY_U3P_ERROR_BAD_ARGUMENT;


    /* Configure Data Format Mode and Value - ConfCTL(0x04) and DataFmt(0x08)*/
    pdfMode = 0x00;

    pdfValue = CY_U3P_MIPICSI_DATAFMT_MASK( (uint8_t) csiCfg->dataFormat );
    pdfMode  = CY_U3P_MIPICSI_DATAMOD_MASK( (uint8_t) csiCfg->dataFormat ) ;

    /* Round up if hResolution is not multiple of 8 */
    if( (csiCfg->hResolution & CY_U3P_MIPICSI_HRES_MIN) > 0)
    {
        if(csiCfg->hResolution > CY_U3P_MIPICSI_HRES_MAX)
            csiCfg->hResolution = CY_U3P_MIPICSI_HRES_MAX; /* Round Down if crosssing max Value*/
        else
            csiCfg->hResolution = (csiCfg->hResolution & CY_U3P_MIPICSI_HRES_MAX) + 8; /* Round Up*/
    }
    switch(csiCfg->dataFormat)
    {
        case CY_U3P_CSI_DF_RAW8:
            wordCount = csiCfg->hResolution;
            break;

        case CY_U3P_CSI_DF_RAW10:
            wordCount = (csiCfg->hResolution * 10)/8;
            break;

        case CY_U3P_CSI_DF_RAW12:
            wordCount = (csiCfg->hResolution * 12)/8;
            break;

        case CY_U3P_CSI_DF_RAW14:
            wordCount = (csiCfg->hResolution * 14)/8;
            break;

        case CY_U3P_CSI_DF_RGB888:
            wordCount = (csiCfg->hResolution * 24)/8;
            break;

        case CY_U3P_CSI_DF_RGB666_0:
        case CY_U3P_CSI_DF_RGB666_1:
            wordCount = (csiCfg->hResolution * 18)/8;
            break;

        case CY_U3P_CSI_DF_RGB565_0:
        case CY_U3P_CSI_DF_RGB565_1:
        case CY_U3P_CSI_DF_RGB565_2:
        case CY_U3P_CSI_DF_YUV422_8_0:
        case CY_U3P_CSI_DF_YUV422_8_1:
        case CY_U3P_CSI_DF_YUV422_8_2:
            wordCount = (csiCfg->hResolution * 16)/8;
            break;

        case  CY_U3P_CSI_DF_YUV422_10:
            wordCount = (csiCfg->hResolution * 20)/8;
            break;

        default:
            status = CY_U3P_ERROR_BAD_ARGUMENT;
            break;
    }
    if(status != CY_U3P_SUCCESS)
        return status;

    /* Validate Number of Data lanes*/
    if ((csiCfg->numDataLanes) > glPartMipiLanes)
    {
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }

    /* Write Register Values*/
    status = CyU3PMipicsiSleep (); /* Turn off the PLL before updating the registers. */
    if (status != CY_U3P_SUCCESS)
        return status;

    CyU3PThreadSleep(1);

    /* 0x16 - CY_U3P_MIPICSI_REG_PLLCTL0*/
    wrBuffer[0] = (((csiCfg->pllPrd) << 4) | (CY_U3P_GET_MSB (csiCfg->pllFbd) & 0x01)) ;
    wrBuffer[1] = (CY_U3P_GET_LSB (csiCfg->pllFbd)) ;

    status = CyU3PMipicsiRegisterWrite (CY_U3P_MIPICSI_REG_PLLCTL0, 2, wrBuffer);
    if (status != CY_U3P_SUCCESS)
        return status;

    /* 0x18 - CY_U3P_MIPICSI_REG_PLLCTL1*/
    wrBuffer[0] = (((csiCfg->pllFrs) << 2) | (CY_U3P_MIPICSI_PLL_LBWS_DEFAULT));
    wrBuffer[1] = (CY_U3P_MIPICSI_PLLCTL1_LOW_INITIAL);

    status = CyU3PMipicsiRegisterWrite (CY_U3P_MIPICSI_REG_PLLCTL1, 2, wrBuffer);
    if (status != CY_U3P_SUCCESS)
        return status;

    CyU3PThreadSleep (1);

    /* 0x18 - CY_U3P_MIPICSI_REG_PLLCTL1*/
    wrBuffer[0] = (((csiCfg->pllFrs) << 2) | (CY_U3P_MIPICSI_PLL_LBWS_DEFAULT));
    wrBuffer[1] = (CY_U3P_MIPICSI_PLLCTL1_LOW_DEFAULT);

    status = CyU3PMipicsiRegisterWrite (CY_U3P_MIPICSI_REG_PLLCTL1, 2, wrBuffer);
    if (status != CY_U3P_SUCCESS)
        return status;


    /* 0x20 - CY_U3P_MIPICSI_REG_CLKCTRL*/
    wrBuffer[0] = 0x00;
    wrBuffer[1] = (((csiCfg->csiRxClkDiv) << 0x04) | ((csiCfg->mClkRefDiv) << 2) | (csiCfg->parClkDiv));

    status = CyU3PMipicsiRegisterWrite (CY_U3P_MIPICSI_REG_CLKCTRL, 2, wrBuffer);
    if (status != CY_U3P_SUCCESS)
        return status;

    /* 0x0C - CY_U3P_MIPICSI_REG_MCLKCTL*/
    wrBuffer[0] = CY_U3P_GET_MSB(csiCfg->mClkCtl);
    wrBuffer[1] = CY_U3P_GET_LSB(csiCfg->mClkCtl);

    status = CyU3PMipicsiRegisterWrite (CY_U3P_MIPICSI_REG_MCLKCTL, 2, wrBuffer);
    if (status != CY_U3P_SUCCESS)
        return status;


    /* 0x22 - CY_U3P_MIPICSI_REG_WORDCNT*/
    wrBuffer[0] = CY_U3P_GET_MSB (wordCount);
    wrBuffer[1] = CY_U3P_GET_LSB (wordCount);

    status = CyU3PMipicsiRegisterWrite (CY_U3P_MIPICSI_REG_WORDCNT, 2, wrBuffer);
    if (status != CY_U3P_SUCCESS)
        return status;
    /* 0x06 - CY_U3P_MIPICSI_REG_FIFOCTL */
    wrBuffer[0] = CY_U3P_GET_MSB (csiCfg->fifoDelay);
    wrBuffer[1] = CY_U3P_GET_LSB (csiCfg->fifoDelay);
    status = CyU3PMipicsiRegisterWrite (CY_U3P_MIPICSI_REG_FIFOCTL, 2, wrBuffer);
    if (status != CY_U3P_SUCCESS)
        return status;
    /* 0x60 - CY_U3P_MIPICSI_REG_PHYTIMDLY  - Only set the upper bit. Do not change this value.*/
    wrBuffer[0] = wrBuffer[1] = 0;
    status = CyU3PMipicsiRegisterRead (CY_U3P_MIPICSI_REG_PHYTIMDLY, 2, wrBuffer);
    if (status != CY_U3P_SUCCESS)
        return status;
    wrBuffer[0] |= 0x80;
    status = CyU3PMipicsiRegisterWrite (CY_U3P_MIPICSI_REG_PHYTIMDLY, 2, wrBuffer);
    if (status != CY_U3P_SUCCESS)
        return status;
    /* 0x08 - CY_U3P_MIPICSI_REG_DATAFMT*/
    wrBuffer[0] = 0x00;
    wrBuffer[1] = ((pdfValue << 4) | glUserDataTypeEnable);

    status = CyU3PMipicsiRegisterWrite (CY_U3P_MIPICSI_REG_DATAFMT, 2, wrBuffer);
    if (status != CY_U3P_SUCCESS)
        return status;

    /* 0x04 - CY_U3P_MIPICSI_REG_CONFCTL*/
    wrBuffer[0] = pdfMode;
    wrBuffer[1] = (((csiCfg->numDataLanes) - 1) | CY_U3P_MIPICSI_CONFCTL_LOW_DEFAULT);

    status = CyU3PMipicsiRegisterWrite (CY_U3P_MIPICSI_REG_CONFCTL, 2, wrBuffer);
    if (status != CY_U3P_SUCCESS)
        return status;

    if(wakeOnConfigure)
        status = CyU3PMipicsiWakeup (); /* Turn on the PLL after updating the registers. */

    return status;
}

/* Get Interface Parameters*/
CyU3PReturnStatus_t
CyU3PMipicsiQueryIntfParams (
        CyU3PMipicsiCfg_t * csiCfg       /* Configuration Data */
        )
{

    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint8_t pdfMode; /* Parallel Data Format Mode Value*/
    uint8_t pdfValue; /* Parallel Data Format. */
    uint32_t bitCount; /* Bit count*/
    uint8_t rdBuffer[2];


    if(!glMipiBlockInitialized)
        return CY_U3P_ERROR_NOT_STARTED;

    if (csiCfg == NULL)
    {
        return CY_U3P_ERROR_NULL_POINTER;
    }

    /* 0x04 - CY_U3P_MIPICSI_REG_CONFCTL*/
    rdBuffer[0] = rdBuffer[1] = 0;
    status = CyU3PMipicsiRegisterRead (CY_U3P_MIPICSI_REG_CONFCTL, 2, rdBuffer);
    if (status != CY_U3P_SUCCESS)
        return status;
    pdfMode = ( (rdBuffer[0] & 0x03) << 0x04);

    csiCfg->numDataLanes = ( (rdBuffer[1] & 0x03) + 1);

    /* 0x08 - CY_U3P_MIPICSI_REG_DATAFMT*/
    rdBuffer[0] = rdBuffer[1] = 0;
    status = CyU3PMipicsiRegisterRead (CY_U3P_MIPICSI_REG_DATAFMT, 2, rdBuffer);
    if (status != CY_U3P_SUCCESS)
        return status;

    pdfValue = ( (rdBuffer[1] & 0xF0) >> 0x04);

    csiCfg ->dataFormat = (CyU3PMipicsiDataFormat_t) (pdfValue | pdfMode);


    /* 0x0C - CY_U3P_MIPICSI_REG_MCLKCTL*/
    rdBuffer[0] = rdBuffer[1] = 0;
    status = CyU3PMipicsiRegisterRead (CY_U3P_MIPICSI_REG_MCLKCTL, 2, rdBuffer);
    if (status != CY_U3P_SUCCESS)
        return status;

    csiCfg->mClkCtl = CY_U3P_MAKEWORD (rdBuffer[0], rdBuffer[1]);


    /* 0x16 - CY_U3P_MIPICSI_REG_PLLCTL0*/
    rdBuffer[0] = rdBuffer[1] = 0;
    status = CyU3PMipicsiRegisterRead (CY_U3P_MIPICSI_REG_PLLCTL0, 2, rdBuffer);
    if (status != CY_U3P_SUCCESS)
        return status;

    csiCfg->pllPrd = ((rdBuffer[0] & 0xF0) >> 4);

    csiCfg->pllFbd = CY_U3P_MAKEWORD ( (rdBuffer[0] &0x01), rdBuffer[1]);

    /* 0x18 - CY_U3P_MIPICSI_REG_PLLCTL1*/
    rdBuffer[0] = rdBuffer[1] = 0;
    status = CyU3PMipicsiRegisterRead (CY_U3P_MIPICSI_REG_PLLCTL1, 2, rdBuffer);
    if (status != CY_U3P_SUCCESS)
        return status;

    csiCfg->pllFrs = (CyU3PMipicsiPllClkFrs_t)((rdBuffer[0] & 0x0C) >> 0x02);


    /* 0x20 - CY_U3P_MIPICSI_REG_CLKCTRL*/
    rdBuffer[0] = rdBuffer[1] = 0;
    status = CyU3PMipicsiRegisterRead (CY_U3P_MIPICSI_REG_CLKCTRL, 2, rdBuffer);
    if (status != CY_U3P_SUCCESS)
        return status;

    csiCfg->csiRxClkDiv  = (CyU3PMipicsiPllClkDiv_t)((rdBuffer[1] & 0x30) >> 0x04);
    csiCfg->mClkRefDiv   = (CyU3PMipicsiPllClkDiv_t)((rdBuffer[1] & 0x0C) >> 0x02);
    csiCfg->parClkDiv    = (CyU3PMipicsiPllClkDiv_t)((rdBuffer[1] & 0x03));

    /* 0x06 - CY_U3P_MIPICSI_REG_FIFOCTL */
    rdBuffer[0] = rdBuffer[1] = 0;
    status = CyU3PMipicsiRegisterRead (CY_U3P_MIPICSI_REG_FIFOCTL, 2, rdBuffer);
    if (status != CY_U3P_SUCCESS)
        return status;
    csiCfg->fifoDelay = (((uint16_t)(rdBuffer[0]&0x01) << 8) | rdBuffer[1]);

    /* 0x22 - CY_U3P_MIPICSI_REG_WORDCNT*/

    rdBuffer[0] = rdBuffer[1] = 0;
    status = CyU3PMipicsiRegisterRead (CY_U3P_MIPICSI_REG_WORDCNT, 2, rdBuffer);
    if (status != CY_U3P_SUCCESS)
        return status;

    bitCount = (CY_U3P_MAKEWORD (rdBuffer[0], rdBuffer[1])) * 8;



    switch(csiCfg->dataFormat)
    {
        case CY_U3P_CSI_DF_RAW8:
            csiCfg->hResolution = bitCount / 8;
            break;

        case CY_U3P_CSI_DF_RAW10:
            csiCfg->hResolution = bitCount / 10 ;
            break;

        case CY_U3P_CSI_DF_RAW12:
            csiCfg->hResolution =  bitCount / 12;
            break;

        case CY_U3P_CSI_DF_RAW14:
            csiCfg->hResolution =  bitCount / 14;
            break;

        case CY_U3P_CSI_DF_RGB888:
            csiCfg->hResolution =  bitCount /  24;
            break;

        case CY_U3P_CSI_DF_RGB666_0:
        case CY_U3P_CSI_DF_RGB666_1:
            csiCfg->hResolution =  bitCount / 18;
            break;

        case CY_U3P_CSI_DF_RGB565_0:
        case CY_U3P_CSI_DF_RGB565_1:
        case CY_U3P_CSI_DF_RGB565_2:
        case CY_U3P_CSI_DF_YUV422_8_0:
        case CY_U3P_CSI_DF_YUV422_8_1:
        case CY_U3P_CSI_DF_YUV422_8_2:
            csiCfg->hResolution =  bitCount /  16;
            break;

        case  CY_U3P_CSI_DF_YUV422_10:
            csiCfg->hResolution = bitCount /  20;
            break;

        default:
            status = CY_U3P_ERROR_BAD_ARGUMENT;
            break;
    }
    return status;
}

/* Set Phy time delay register */
extern CyU3PReturnStatus_t
CyU3PMipicsiSetPhyTimeDelay  (

        uint8_t tdTerm,                 /**< TD TERM Selection for MIPI CSI-2 reciever PHY. 
                                          Valid values 0 and 1. */
        uint8_t thsSettleDelay          /**< THS Settle timer delay value. 
                                          Valid range 0x00-0x7F. */
        )
{
    /* 0x60 - CY_U3P_MIPICSI_REG_PHYTIMDLY  - Only set the upper bit. Do not change this value.*/
    uint8_t wrBuffer[2];
    CyU3PReturnStatus_t status;
    

    wrBuffer[0] = wrBuffer[1] = 0;

    if(!glMipiBlockInitialized)
        return CY_U3P_ERROR_NOT_STARTED;
    
    if ((tdTerm > 1) || (thsSettleDelay > 0x7F))
    {
        return CY_U3P_ERROR_BAD_ARGUMENT; /* Invalid value passed in*/
    }
    status = CyU3PMipicsiRegisterRead (CY_U3P_MIPICSI_REG_PHYTIMDLY, 2, wrBuffer);
    if (status != CY_U3P_SUCCESS)
        return status;
    wrBuffer[0] |= 0x80; /* Set TC_TERM */
    wrBuffer[1] = CY_U3P_MIPICSI_THS_PHY_DELAY_MASK(tdTerm, thsSettleDelay); /* Set TD Term and 
                                                                                THS Settle values*/
    status = CyU3PMipicsiRegisterWrite (CY_U3P_MIPICSI_REG_PHYTIMDLY, 2, wrBuffer);
    return status;
}


/* User level reset for MIPI block. Allows Hard or Soft reset*/

CyU3PReturnStatus_t
CyU3PMipicsiReset (
        CyU3PMipicsiReset_t resetType)
{
    CyU3PReturnStatus_t status;
    switch (resetType)
    {
        case CY_U3P_CSI_SOFT_RST:
            if (glMipiBlockInitialized) /* Register write will fail if block is not initialized. */
            {
                glMipiBlockInitialized = CyFalse;
                status = CyU3PMipicsiSoftReset(CyFalse);
            }
            else
                status = CY_U3P_ERROR_NOT_STARTED;
            break;
        case CY_U3P_CSI_HARD_RST:
            glMipiBlockInitialized = CyFalse;   /* Dont check for initialized on hard reset. */
            status = CyU3PMipicsiHardReset(CyFalse);
            break;
        default:
            status = CY_U3P_ERROR_BAD_ARGUMENT;
            break;
    }
    if(status != CY_U3P_SUCCESS)
    {
        return status;
    }

    CyU3PThreadSleep(5); /* 5 ms dealy to allow the MIPI block to stabilize after reset is completed*/

    status = CyU3PMipicsiVerifyBlockId();
    if( status == CY_U3P_SUCCESS)
    {
        glMipiBlockInitialized = CyTrue;
    }

    return status;
}


CyU3PReturnStatus_t
CyU3PMipicsiGetErrors (
        CyBool_t clrErrCnts,                        /* Set to CyTrue to clear the error counts*/
        CyU3PMipicsiErrorCounts_t * errorCounts    /* Error Counts*/
        )
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint8_t rwBuffer[2];
    uint16_t csiStatus;
    uint16_t i, regAddr;
    uint8_t * errCntMember;
    if(!glMipiBlockInitialized)
        return CY_U3P_ERROR_NOT_STARTED;

    if (errorCounts == NULL)
    {
        return CY_U3P_ERROR_NULL_POINTER;
    }
    CyU3PMemSet((uint8_t*)errorCounts, 0x00, sizeof(CyU3PMipicsiErrorCounts_t));

    /* Read the Error status register */
    regAddr = 0;

    rwBuffer[0] = rwBuffer[1] = 0;

    status = CyU3PMipicsiRegisterRead(CY_U3P_MIPICSI_REG_CSI_STAT, 2, rwBuffer);
    if (status != CY_U3P_SUCCESS)
        return status;

    csiStatus =  (CY_U3P_MAKEWORD (rwBuffer[0], rwBuffer[1]));

    if (csiStatus > 0)
    {
        /* Get/Clear error Counts*/
        for (i = 0x0001; i < 0x0200; i = (i << 1))
        {
            switch(csiStatus & i)
            {
                case CY_U3P_MIPICSI_SYNERR_MASK:                    /* 0x0001 */
                    regAddr = CY_U3P_MIPICSI_REG_SYN_ERR_CNT;
                    errCntMember = & errorCounts->unrSyncErrCnt;
                    break;
                case CY_U3P_MIPICSI_SOTERR_MASK:                    /* 0x0002 */
                    regAddr = CY_U3P_MIPICSI_REG_SOT_ERR_CNT;
                    errCntMember = & errorCounts->recSyncErrCnt;
                    break;
                case CY_U3P_MIPICSI_CTLERR_MASK:                    /* 0x0004 */
                    regAddr = CY_U3P_MIPICSI_REG_CTL_ERR_CNT;
                    errCntMember = & errorCounts->ctlErrCnt;
                    break;
                case CY_U3P_MIPICSI_EIDERR_MASK:                    /* 0x0008 */

                    regAddr = CY_U3P_MIPICSI_REG_EID_ERR_CNT;
                    errCntMember = & errorCounts->eidErrCnt;
                    break;
                case CY_U3P_MIPICSI_HDRERR_MASK:                    /* 0x0010 */

                    regAddr = CY_U3P_MIPICSI_REG_HDR_ERR_CNT;
                    errCntMember = & errorCounts->unrcErrCnt;
                    break;
                case CY_U3P_MIPICSI_CORERR_MASK:                    /* 0x0020 */

                    regAddr = CY_U3P_MIPICSI_REG_COR_ERR_CNT;
                    errCntMember = & errorCounts->recrErrCnt;
                    break;
                case CY_U3P_MIPICSI_CRCERR_MASK:                    /* 0x0040 */

                    regAddr = CY_U3P_MIPICSI_REG_CRC_ERR_CNT;
                    errCntMember = & errorCounts->crcErrCnt;
                    break;
                case CY_U3P_MIPICSI_FRMERR_MASK:                    /* 0x0080 */

                    regAddr = CY_U3P_MIPICSI_REG_FRM_ERR_CNT;
                    errCntMember = & errorCounts->frmErrCnt;
                    break;
                case CY_U3P_MIPICSI_MDLERR_MASK:                    /* 0x0100 */

                    regAddr = CY_U3P_MIPICSI_REG_MDL_ERR_CNT;
                    errCntMember = & errorCounts->mdlErrCnt;
                    break;
                default:
                    continue;
            } /* End Switch */

            rwBuffer[0] = rwBuffer[1] = 0;

            status = CyU3PMipicsiRegisterRead(regAddr, 2, rwBuffer);
            if (status != CY_U3P_SUCCESS)
                return status;

            if(errCntMember != NULL)
                *errCntMember = rwBuffer[1];

            if (clrErrCnts)
            {
                rwBuffer[0] = rwBuffer[1] = 0;
                status = CyU3PMipicsiRegisterWrite(regAddr, 2, rwBuffer);
                if (status != CY_U3P_SUCCESS)
                    return status;
            }

        } /* End For */

        /* Clear the Error Status register by W1C */
        rwBuffer[0] = (CY_U3P_GET_MSB (csiStatus)) & 0x01;
        rwBuffer[1] = (CY_U3P_GET_LSB (csiStatus)) & 0xFF;

        status = CyU3PMipicsiRegisterWrite(CY_U3P_MIPICSI_REG_CSI_STAT, 2, rwBuffer);
        if (status != CY_U3P_SUCCESS)
            return status;


    }/* End if (csiStatus >0) */


    return status;
}

/* Wakeup the MIPI block*/
CyU3PReturnStatus_t
CyU3PMipicsiWakeup (
        void)
{
    uint8_t buffer[2];
    CyU3PReturnStatus_t status;
    if(!glMipiBlockInitialized)
        return CY_U3P_ERROR_NOT_STARTED;
    /* Check if the block is already Powerd Up */
    if(glIsMipiBlockActive)
        return CY_U3P_SUCCESS;

    buffer[0] = buffer[1] = 0;
    status = CyU3PMipicsiRegisterRead(CY_U3P_MIPICSI_REG_SYSCTL, 2, buffer);
    if (status != CY_U3P_SUCCESS)
        return status;

    /* Put the block into Reset */
    buffer[1] &= ~(CY_U3P_MIPICSI_BLOCK_SLEEP);
    status = CyU3PMipicsiRegisterWrite (CY_U3P_MIPICSI_REG_SYSCTL, 2, buffer);
    if( status != CY_U3P_SUCCESS)
        return status;
    else
        glIsMipiBlockActive = CyTrue;
    return status;
}

/* Set MIPI block to sleep . Clocks are disabled and data transmission is stopped*/
CyU3PReturnStatus_t
CyU3PMipicsiSleep (
        void)
{
    uint8_t buffer[2];
    CyU3PReturnStatus_t status;
    if(!glMipiBlockInitialized)
        return CY_U3P_ERROR_NOT_STARTED;
    /* Check if Block is already Powered Down */
    if(!glIsMipiBlockActive)
        return CY_U3P_SUCCESS;
    buffer[0] = buffer[1] = 0;
    status = CyU3PMipicsiRegisterRead(CY_U3P_MIPICSI_REG_SYSCTL, 2, buffer);
    if (status != CY_U3P_SUCCESS)
        return status;

    /* Put the block into Reset */
    buffer[1] |= CY_U3P_MIPICSI_BLOCK_SLEEP;
    status = CyU3PMipicsiRegisterWrite (CY_U3P_MIPICSI_REG_SYSCTL, 2, buffer);
    if( status != CY_U3P_SUCCESS)
        return status;
    CyU3PThreadSleep(5);

    buffer[0] = buffer[1] = 0;
    status = CyU3PMipicsiRegisterRead(CY_U3P_MIPICSI_REG_SYSCTL, 2, buffer);
    if (status != CY_U3P_SUCCESS)
        return status;
    else if ((buffer[1] & CY_U3P_MIPICSI_BLOCK_SLEEP) == CY_U3P_MIPICSI_BLOCK_SLEEP)
        glIsMipiBlockActive = CyFalse;
    else
        status = CY_U3P_ERROR_FAILURE;

    return status;
}

/* Check if the block is active or in Sleep Mode*/
CyBool_t CyU3PMipicsiCheckBlockActive()
{
    return glIsMipiBlockActive;
}




/* Control the Sensor Reset IOs*/
CyU3PReturnStatus_t
CyU3PMipicsiSetSensorControl (
        CyU3PMipicsiSensorIo_t io,  /* Select IO. */
        CyBool_t value              /* CyTrue for High, CyFalse for Low. */
        )
{

    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    /* Set GPIOs to Output */

    uint8_t rwBuffer[2];
    uint8_t wrVal;
    rwBuffer[0] = rwBuffer[1] = 0x00;

    if(!glMipiBlockInitialized)
        return CY_U3P_ERROR_NOT_STARTED;

    /* Clear all bits except the two for XRES and XSHUTDOWN */
    io &= (CY_U3P_CSI_IO_XRES | CY_U3P_CSI_IO_XSHUTDOWN);

    if (((uint8_t)io) == 0)
    {
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }


    /* Read the current State of the GPIOs */
    status = CyU3PMipicsiRegisterRead (CY_U3P_MIPICSI_REG_GPIOIN, 2, rwBuffer);
    if(status != CY_U3P_SUCCESS)
        return status;

    if(value)
    {
        /* Set IO(s) High. */

        rwBuffer[1] |= ((uint8_t) io);
    }
    else
    {
        /* Set selected IO(s) low */
        rwBuffer[1] &= ~((uint8_t) io);
    }
    wrVal = rwBuffer[1];

    status = CyU3PMipicsiRegisterWrite (CY_U3P_MIPICSI_REG_GPIOOUT, 2, rwBuffer);
    if(status != CY_U3P_SUCCESS)
        return status;

    CyU3PThreadSleep(5);
    rwBuffer [0] = rwBuffer[1] = 0;

    status = CyU3PMipicsiRegisterRead (CY_U3P_MIPICSI_REG_GPIOIN, 2, rwBuffer);
    if(status != CY_U3P_SUCCESS)
        return status;
    if ( rwBuffer[1] != wrVal)
    {
        status = CY_U3P_ERROR_FAILURE;
    }
    return status;
}


/* Device Reset along with IS and MIPI block reset. This function will not return*/
void
CyU3PCx3DeviceReset(
        CyBool_t isWarmReset,       /*  Whether this should be a warm reset or a cold reset. In the case of a warm
                                        reset, the previously loaded firmware will start executing again. In the
                                        case of cold reset, the firmware download to CX3 needs to be performed again. */

        CyBool_t sensorResetHigh    /*  True if Sensor is to be reset by driving XRES High,
                                        False if Sensor reset needs XRES as low. */
        )
{
    CyU3PMipicsiSetSensorControl(CY_U3P_CSI_IO_XRES, sensorResetHigh); /* Signal a reset to the Image Sensor*/

    CyU3PThreadSleep(1);

    CyU3PMipicsiSoftReset (CyTrue); /* Put Mipi Block into reset. Do not clear reset in call */

    CyU3PThreadSleep(1);

    glMipiBlockInitialized = glIsMipiBlockActive = CyFalse; /* Reset the initialization flags before Reset device*/

    CyU3PDeviceReset (isWarmReset); /* Reset the CX3  - This call does not return.*/
}

/**************************************/
/*          Helper Routines         */
/*************************************/

/* Initialize the GPIO block on the CX3.
   Should be called before initializing PIB block or calling MIPI Init*/
    CyU3PReturnStatus_t
CyU3PMipicsiInitializeGPIO (void)
{
    CyU3PGpioClock_t gpioClock;

    /* Initialize the GPIO module */
    gpioClock.fastClkDiv = 2;
    gpioClock.slowClkDiv = 0;
    gpioClock.simpleDiv = CY_U3P_GPIO_SIMPLE_DIV_BY_2;
    gpioClock.clkSrc = CY_U3P_SYS_CLK;
    gpioClock.halfDiv = 0;
    return CyU3PGpioInit(&gpioClock, NULL);
}


/* Configure and Initialize the I2C Block onthe CX3
   Should be called before calling CyU3PMipicsiInit() */
CyU3PReturnStatus_t
CyU3PMipicsiInitializeI2c (
        CyU3PMipicsiI2cFreq_t freq  /* Frequency 100KHz or 400KHz */
        )
{
    CyU3PI2cConfig_t i2cConfig;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Initialize and configure the I2C master module.
       If the block is already started go ahead and set
       configuration to override any preset configuration*/
    status = CyU3PI2cInit ();
    if ((status != CY_U3P_SUCCESS) && (status != CY_U3P_ERROR_ALREADY_STARTED))
    {
        return status;
    }


    /* Start the I2C master block. The bit rate is set to 100KHz or 400KHz. */
    CyU3PMemSet ((uint8_t *)&i2cConfig, 0, sizeof(i2cConfig));
    i2cConfig.bitRate = 100000;
    switch (freq)
    {
        case CY_U3P_MIPICSI_I2C_100KHZ:
            i2cConfig.bitRate = 100000;
            break;
        case CY_U3P_MIPICSI_I2C_400KHZ:
            i2cConfig.bitRate = 400000;
            break;
        default:
            return CY_U3P_ERROR_BAD_ARGUMENT;
    }
    i2cConfig.busTimeout = 0xFFFFFFFF;
    i2cConfig.dmaTimeout = 0xFFFF;
    i2cConfig.isDma      = CyFalse;
    status = CyU3PI2cSetConfig (&i2cConfig, NULL);

    return status;
}


/* Initialize and configure the PIB block on the CX3.
   Should be called before initializing the GPIF uisng CyU3PMipicsiGpifLoad()*/
CyU3PReturnStatus_t
CyU3PMipicsiInitializePIB (
        void)
{
    CyU3PPibClock_t pibClock;

    /* Initialize the p-port block. */
    pibClock.clkDiv = 2;
    pibClock.clkSrc = CY_U3P_SYS_CLK;
    pibClock.isHalfDiv = CyFalse;
    /* Disable DLL for sync GPIF */
    pibClock.isDllEnable = CyFalse;
    return CyU3PPibInit(CyTrue, &pibClock);
}

/*[]*/

