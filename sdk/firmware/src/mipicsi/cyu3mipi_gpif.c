/*
 ## Cypress CX3 MIPI-CSI Driver Source (cyu3mipi_gpif.c)
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

/*@@MIPI GPIF Functions
 * This file contains the GPIF Waveform data and the GPIF load function 
 * for the CX3 parts which only support a fixed function GPIF interfcae
 */

#include <cyu3pib.h>
#include <cywbpib.h>
#include <cyu3gpif.h>
#include <cyu3error.h>
#include <cyu3regs.h>
#include <cyu3utils.h>
#include <cyu3system.h>
#include <cyu3types.h>
#include <cyu3mipicsi.h>
#include <cyfx3_api.h>

#define CY_U3P_CSI_GPIF_8BIT_PARAM          (0x00000003)
#define CY_U3P_CSI_GPIF_16BIT_PARAM         (0x00000067)
#define CY_U3P_CSI_GPIF_24BIT_PARAM         (0x00000009)

#define CY_U3P_CSI_GPIF_BUS_WIDTH_POS       (0x01)
#define CY_U3P_CSI_GPIF_DATA_COUNT_POS      (0x27)

/* Summary
   Transition function values used in the state machine.
 */
uint16_t CyCx3GpifTransition[]  = {
    0x0000, 0xAAAA, 0x5555, 0x8888, 0x3333
};

/* Summary
   Table containing the transition information for various states. 
   This table has to be stored in the WAVEFORM Registers.
   This array consists of non-replicated waveform descriptors and acts as a 
   waveform table. 
 */
CyU3PGpifWaveData CyCx3GpifWavedata[]  = {
    {{0x2E738C01,0x00000000,0x80000000},{0x00000000,0x00000000,0x00000000}},
    {{0x3E716C02,0x00000100,0x80000080},{0x00000000,0x00000000,0x00000000}},
    {{0x1E739403,0x20000104,0x80000040},{0x00000000,0x00000000,0x00000000}},
    {{0x1E718B06,0x00000108,0x80000000},{0x1E718B05,0x00000108,0x80000000}},
    {{0x1E718B08,0x00000108,0x80000000},{0x1E718B07,0x00000108,0x80000000}},
    {{0x1E739403,0x20000104,0x80000040},{0x00000009,0x00000000,0x80000100}},
    {{0x1E739404,0x24000104,0x80000040},{0x0000000B,0x00000000,0x80000100}},
    {{0x1E739404,0x24000104,0x80000040},{0x0000000A,0x00000000,0x80000100}},
    {{0x1E739403,0x20000104,0x80000040},{0x0000000C,0x00000000,0x80000100}},
    {{0x00000000,0x00000000,0x00000000},{0x00000000,0x00000000,0x00000000}},
    {{0x2E738C0E,0x00000000,0x80000000},{0x00000000,0x00000000,0x00000000}},
    {{0x3E716C0F,0x00000100,0x80000080},{0x00000000,0x00000000,0x00000000}},
    {{0x1E739404,0x24000104,0x80000040},{0x00000000,0x00000000,0x00000000}}
};

/* Summary
   Table that maps state indices to the descriptor table indices.
 */
uint8_t CyCx3GpifWavedataPosition[]  = {
    0,1,2,3,4,5,6,7,8,9,9,9,9,10,11,12,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,
    9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,
    9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,
    9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,
    0,1,2,12,2,5,6,7,8,9,9,9,9,10,11,12
};

/* Summary
   GPIF II configuration register values.
 */
uint32_t CyCx3GpifRegValue[]  = {
    0x80008300,  /*  CY_U3P_PIB_GPIF_CONFIG */
    0x00000067,  /*  CY_U3P_PIB_GPIF_BUS_CONFIG */
    0x0B000001,  /*  CY_U3P_PIB_GPIF_BUS_CONFIG2 */
    0x00000046,  /*  CY_U3P_PIB_GPIF_AD_CONFIG */
    0x00000000,  /*  CY_U3P_PIB_GPIF_STATUS */
    0x00000000,  /*  CY_U3P_PIB_GPIF_INTR */
    0x00000002,  /*  CY_U3P_PIB_GPIF_INTR_MASK */
    0x00000082,  /*  CY_U3P_PIB_GPIF_SERIAL_IN_CONFIG */
    0x00000782,  /*  CY_U3P_PIB_GPIF_SERIAL_OUT_CONFIG */
    0x00100000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_DIRECTION */
    0x0000E7FF,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_DEFAULT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_POLARITY */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_TOGGLE */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000006,  /*  CY_U3P_PIB_GPIF_CTRL_COUNT_CONFIG */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_COUNT_RESET */
    0x0000FFFF,  /*  CY_U3P_PIB_GPIF_CTRL_COUNT_LIMIT */
    0x0000010A,  /*  CY_U3P_PIB_GPIF_ADDR_COUNT_CONFIG */
    0x00000000,  /*  CY_U3P_PIB_GPIF_ADDR_COUNT_RESET */
    0x0000FFFF,  /*  CY_U3P_PIB_GPIF_ADDR_COUNT_LIMIT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_STATE_COUNT_CONFIG */
    0x0000FFFF,  /*  CY_U3P_PIB_GPIF_STATE_COUNT_LIMIT */
    0x0000010B,  /*  CY_U3P_PIB_GPIF_DATA_COUNT_CONFIG */
    0x00000000,  /*  CY_U3P_PIB_GPIF_DATA_COUNT_RESET */
    0x000017F7,  /*  CY_U3P_PIB_GPIF_DATA_COUNT_LIMIT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_COMP_VALUE */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_COMP_MASK */
    0x00000000,  /*  CY_U3P_PIB_GPIF_DATA_COMP_VALUE */
    0x00000000,  /*  CY_U3P_PIB_GPIF_DATA_COMP_MASK */
    0x00000000,  /*  CY_U3P_PIB_GPIF_ADDR_COMP_VALUE */
    0x00000000,  /*  CY_U3P_PIB_GPIF_ADDR_COMP_MASK */
    0x00000000,  /*  CY_U3P_PIB_GPIF_DATA_CTRL */
    0x00000000,  /*  CY_U3P_PIB_GPIF_INGRESS_DATA */
    0x00000000,  /*  CY_U3P_PIB_GPIF_INGRESS_DATA */
    0x00000000,  /*  CY_U3P_PIB_GPIF_INGRESS_DATA */
    0x00000000,  /*  CY_U3P_PIB_GPIF_INGRESS_DATA */
    0x00000000,  /*  CY_U3P_PIB_GPIF_EGRESS_DATA */
    0x00000000,  /*  CY_U3P_PIB_GPIF_EGRESS_DATA */
    0x00000000,  /*  CY_U3P_PIB_GPIF_EGRESS_DATA */
    0x00000000,  /*  CY_U3P_PIB_GPIF_EGRESS_DATA */
    0x00000000,  /*  CY_U3P_PIB_GPIF_INGRESS_ADDRESS */
    0x00000000,  /*  CY_U3P_PIB_GPIF_INGRESS_ADDRESS */
    0x00000000,  /*  CY_U3P_PIB_GPIF_INGRESS_ADDRESS */
    0x00000000,  /*  CY_U3P_PIB_GPIF_INGRESS_ADDRESS */
    0x00000000,  /*  CY_U3P_PIB_GPIF_EGRESS_ADDRESS */
    0x00000000,  /*  CY_U3P_PIB_GPIF_EGRESS_ADDRESS */
    0x00000000,  /*  CY_U3P_PIB_GPIF_EGRESS_ADDRESS */
    0x00000000,  /*  CY_U3P_PIB_GPIF_EGRESS_ADDRESS */
    0x80010400,  /*  CY_U3P_PIB_GPIF_THREAD_CONFIG */
    0x80010401,  /*  CY_U3P_PIB_GPIF_THREAD_CONFIG */
    0x80010402,  /*  CY_U3P_PIB_GPIF_THREAD_CONFIG */
    0x80010403,  /*  CY_U3P_PIB_GPIF_THREAD_CONFIG */
    0x00000000,  /*  CY_U3P_PIB_GPIF_LAMBDA_STAT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_ALPHA_STAT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_BETA_STAT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_WAVEFORM_CTRL_STAT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_WAVEFORM_SWITCH */
    0x00000000,  /*  CY_U3P_PIB_GPIF_WAVEFORM_SWITCH_TIMEOUT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CRC_CONFIG */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CRC_DATA */
    0xFFFFFFC1   /*  CY_U3P_PIB_GPIF_BETA_DEASSERT */
};

/* Summary
   This structure holds all the configuration inputs for the GPIF II. 
 */
const CyU3PGpifConfig_t CyCx3GpifConfig  = {
        (uint16_t)(sizeof(CyCx3GpifWavedataPosition)/sizeof(uint8_t)),
        CyCx3GpifWavedata,
        CyCx3GpifWavedataPosition,
        (uint16_t)(sizeof(CyCx3GpifTransition)/sizeof(uint16_t)),
        CyCx3GpifTransition,
        (uint16_t)(sizeof(CyCx3GpifRegValue)/sizeof(uint32_t)),
        CyCx3GpifRegValue
        };

/* Configure the GPIF Config Data structure and Call the GpifLoad() */
CyU3PReturnStatus_t Cx3LoadGpifConfigData (
        void)
{
    return CyU3PGpifLoad(&CyCx3GpifConfig);
}

/* GPIF Load Routine for CX3 */
CyU3PReturnStatus_t 
CyU3PMipicsiGpifLoad (
        CyU3PMipicsiBusWidth_t busWidth,
        uint32_t               bufferSize )
{
    /* CX3 Disable Check */
    if (!(CyFx3DevIsMipicsiSupported ()))
    {
        return CY_U3P_ERROR_NOT_SUPPORTED;
    }
    
    /* Update the GPIF Waveform based on GPIF width requested. */
    switch (busWidth)
    {
        case CY_U3P_MIPICSI_BUS_8:
           CyCx3GpifRegValue[CY_U3P_CSI_GPIF_BUS_WIDTH_POS] = CY_U3P_CSI_GPIF_8BIT_PARAM;
           CyCx3GpifRegValue[CY_U3P_CSI_GPIF_DATA_COUNT_POS] = bufferSize-1;
           break;
        case CY_U3P_MIPICSI_BUS_16:
           CyCx3GpifRegValue[CY_U3P_CSI_GPIF_BUS_WIDTH_POS] = CY_U3P_CSI_GPIF_16BIT_PARAM;
           if( (bufferSize & 0x1) != 0)
               return CY_U3P_ERROR_BAD_ARGUMENT;
           CyCx3GpifRegValue[CY_U3P_CSI_GPIF_DATA_COUNT_POS] = (bufferSize / 2) - 1;
           break;
        case CY_U3P_MIPICSI_BUS_24:
           CyCx3GpifRegValue[CY_U3P_CSI_GPIF_BUS_WIDTH_POS] = CY_U3P_CSI_GPIF_24BIT_PARAM;
           
           if( (bufferSize % 3) != 0)
               return CY_U3P_ERROR_BAD_ARGUMENT;
           CyCx3GpifRegValue[CY_U3P_CSI_GPIF_DATA_COUNT_POS] = (bufferSize / 3) - 1;
           break;
       default:
           return CY_U3P_ERROR_BAD_ARGUMENT;
    }
    
    return Cx3LoadGpifConfigData();
}

/*[]*/

