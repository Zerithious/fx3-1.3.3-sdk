/*
 ## Cypress FX3 Boot Firmware Header (cyfx3pib.h)
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

#ifndef _INCLUDED_CYFX3PIB_H_
#define _INCLUDED_CYFX3PIB_H_

#include <cyfx3error.h>
#include <cyu3types.h>
#include <cyfx3device.h>
#include <cyu3externcstart.h>

/** \file cyfx3pib.h
    \brief The PIB driver module in the FX3 boot firmware enables the PMMC
    interface on the FX3 device and allows PMMC data transfers.
 */

/**************************************************************************
 ********************************* Macros *********************************
 **************************************************************************/

/** \def CYFX3_GPIF_NUM_STATES
    \brief Number of states supported by the GPIF hardware.
 */
#define CYFX3_GPIF_NUM_STATES                           (256)

/** \def CYFX3_GPIF_NUM_TRANS_FNS
    \brief Number of distinct transfer functions supported by the GPIF hardware.
 */
#define CYFX3_GPIF_NUM_TRANS_FNS                        (32)

/** \def CYFX3_GPIF_INVALID_STATE
    \brief Invalid state index for use in state machine control functions.
 */
#define CYFX3_GPIF_INVALID_STATE                        (0xFFFF)

/** \def CYFX3_PIB_THREAD_COUNT
    \brief Number of DMA threads on the GPIF (P-Port)
 */
#define CYFX3_PIB_THREAD_COUNT                          (4)

/** \def CYFX3_PIB_SOCKET_COUNT
    \brief Number of DMA sockets on the GPIF (P-port)
 */
#define CYFX3_PIB_SOCKET_COUNT                          (32)

/** \def CYFX3_PIB_MAX_BURST_SETTING
    \brief Maximum burst size allowed for P-port sockets. The constant corresponds
    to Log(2) of size, which means that the max. size is 16 KB.
 */
#define CYFX3_PIB_MAX_BURST_SETTING                     (14)

/**************************************************************************
 ******************************* Data Types *******************************
 **************************************************************************/

/** \brief Clock configuration information for the PIB block.

    **Description**\n
    The clock for the PIB block is derived from the SYS_CLK. The base clock
    and frequency divider values are selected through this structure.

    **\see
    *\see CyFx3BootSysClockSrc_t
    *\see CyFx3BootPibInit
 */
typedef struct CyFx3BootPibClock_t
{
    CyFx3BootSysClockSrc_t clkSrc;      /**< The clock source to be used. */
    uint16_t               clkDiv;      /**< Divider for the PIB clock. The min value is 2 and max value is 1024. */
    CyBool_t               isHalfDiv;   /**< If set to true, adds 0.5 to the divider value selected by clkDiv. */
    CyBool_t               isDllEnable; /**< Whether the DLL should be enabled or not. The DLL in the PIB block should
                                             be enabled when implementing Asynchronous GPIF protocols, or Master mode
                                             GPIF protocols. It should be left turned off when implementing synchronous
                                             slave mode GPIF protocols. */
} CyFx3BootPibClock_t;

/** \brief List of MMC-Slave Event notifications.

    **Description**\n
    This type lists the various event notifications that are provided by the FX3
    device when configured in the MMC-Slave mode.

    **\see\n
    *\see CyFx3BootPibRegisterMmcCallback
 */
typedef enum CyFx3BootPMMCEvent_t
{
    CYFX3_PMMC_GOIDLE_CMD = 0,          /**< GO_IDLE_STATE (CMD0) command has been received. */
    CYFX3_PMMC_CMD5_SLEEP,              /**< CMD5(SLEEP) command has been used to place FX3 into suspend mode. */
    CYFX3_PMMC_CMD5_AWAKE,              /**< CMD5(AWAKE) command has been used to wake FX3 from suspend mode. */
    CYFX3_PMMC_CMD6_SWITCH,             /**< SWITCH (CMD6) command has been received. */
    CYFX3_PMMC_CMD12_STOP,              /**< STOP_TRANSMISSION (CMD12) command has been received. */
    CYFX3_PMMC_CMD15_INACTIVE,          /**< GO_INACTIVE_STATE (CMD15) command has been received. */
    CYFX3_PMMC_SOCKET_NOT_READY,        /**< Socket being read/written by the host is not ready. */
    CYFX3_PMMC_CMD7_SELECT              /**< SELECT_CARD (CMD7) command has been received. */
} CyFx3BootPMMCEvent_t;

/** \brief Callback function type used for MMC-Slave event notifications.

    **Description**\n
    This function pointer type defines the signature of the callback function
    that will be called to notify the user of MMC-Slave (PMMC) Mode events.

    **\see\n
    *\see CyFx3BootPMMCEvent_t
    *\see CyFx3BootPibRegisterMmcCallback
 */
typedef void (*CyFx3BootPMMCIntrCb_t) (
        CyFx3BootPMMCEvent_t cbType,    /**< Type of PMMC event being notified. */
        uint32_t             cbArg      /**< 32-bit integer argument associated with the interrupt. This
                                             commonly represents the command argument received from the host. */
        );

/** \brief Callback function type to be called when a GPIF state machine interrupt is triggered.

    **Description**\n
    This type defines the type of a callback function that will be called when the GPIF state machine
    triggers an interrupt.

    **\see
    *\see CyFx3BootGpifRegisterCallback
 */
typedef void (*CyFx3BootGpifIntrCb_t) (
        uint8_t currentState                    /**< State from which the interrupt is triggered. */
        );

/** \brief Information on a single GPIF transition from one state to another.

    **Description**\n
    The GPIF state machine on the FX3 device is defined through a
    set of transition descriptors. These descriptors include fields for
    specifying the next state, the conditions for transition, and the
    output values.

    This structure encapsulates all of the information that forms the left
    and right transition descriptors for a state.

    **\see
    *\see CyU3PGpifConfig_t
 */
typedef struct CyU3PGpifWaveData
{
    uint32_t leftData[3];                       /**< 12 byte left transition descriptor. */
    uint32_t rightData[3];                      /**< 12 byte right transition descriptor. */
} CyU3PGpifWaveData;

/** \brief Structure that holds all configuration inputs for the GPIF hardware.

    **Description**\n
    The GPIF block on the FX3 device has a set of general configuration registers,
    transition function registers and state descriptors that need to be initialized
    to make the GPIF state machine functional. This structure encapsulates all the
    data that is required to program the GPIF block to load a user defined state
    machine.

    The GPIF configuration data in the form of this structure is commonly generated
    by the GPIF II Designer tool.

    **\see
    *\see CyU3PGpifWaveData
    *\see CyFx3BootGpifLoad
 */
typedef struct CyU3PGpifConfig_t
{
    const uint16_t           stateCount;        /**< Number of states to be initialized. */
    const CyU3PGpifWaveData *stateData;         /**< Pointer to array containing state descriptors. */
    const uint8_t           *statePosition;     /**< Pointer to array index -> state number mapping. */
    const uint16_t           functionCount;     /**< Number of transition functions to be initialized. */
    const uint16_t          *functionData;      /**< Pointer to array containing transition function data. */
    const uint16_t           regCount;          /**< Number of GPIF config registers to be initialized. */
    const uint32_t          *regData;           /**< Pointer to array containing GPIF register values. */
} CyU3PGpifConfig_t;

/** \brief List of GPIF counters.

    **Description**\n
    The GPIF II hardware provides a set of counters that can be used to control the state machine
    operation. This type lists the various types of counters supported by the GPIF hardware.

    **\see
    *\see CyFx3BootGpifInitCounter
 */
typedef enum CyFx3GpifCounterType
{
    CYFX3_GPIF_COUNTER_CONTROL = 0,             /**< Control counter: 16 bit. */
    CYFX3_GPIF_COUNTER_ADDRESS,                 /**< Address counter: 32 bit. */
    CYFX3_GPIF_COUNTER_DATA                     /**< Data counter: 32 bit. */
} CyFx3GpifCounterType;

/**************************************************************************
 *************************** Function prototypes **************************
 **************************************************************************/

/** \brief Starts the Processor Interface Block

    **Description**\n
    Initialize the Processor Interface Block on the FX3 device. The clock
    parameters and the interface mode (PMMC or GPIF-II) are to be specified
    as parameters.

    **Return value**\n
    * CY_FX3_BOOT_SUCCESS             - If the block is initialized successfully.\n
    * CY_FX3_BOOT_ERROR_NOT_SUPPORTED - If the mode selected is not supported.

    **\see
    *\see CyFx3BootPibDeinit
 */
extern CyFx3BootErrorCode_t
CyFx3BootPibInit (
        CyFx3BootPibClock_t *clkInfo_p,         /**< Selected clock parameters. */
        CyBool_t             isMmcMode          /**< Whether to start in PMMC (CyTrue) or GPIF-II (CyFalse) mode. */
        );

/** \brief Stops the Processor Interface Block

    **Description**\n
    This function disables and powers off the PIB block on FX3.

    **Return value**\n
    * CY_FX3_BOOT_SUCCESS           - If the block is stopped successfully.\n
    * CY_FX3_BOOT_ERROR_NOT_STARTED - If the block has not been initialized.

    **\see
    *\see CyFx3BootPibInit
 */
extern CyFx3BootErrorCode_t
CyFx3BootPibDeinit (
        void);

/** \brief Register a callback for notification of PMMC interface events.

    **Description**\n
    Set a callback function to receive notification of PMMC interface events.
    These are typically invoked when the MMC host sends down specific commands
    that require firmware intervention.

    **\see
    *\see CyFx3BootPMMCEvent_t
    *\see CyFx3BootPMMCIntrCb_t
 */
extern void
CyFx3BootPibRegisterMmcCallback (
        CyFx3BootPMMCIntrCb_t cb                /**< Callback function pointer. */
        );

/** \brief PIB event handler function.

    **Description**\n
    This function looks for PIB related interrupts and handles them. This function
    is expected to be called periodically from the application main, if the PIB
    block is being used.

    **Return Value**\n
    * None
 */
extern void
CyFx3BootPibHandleEvents (
        void);

/** \brief This function is used to setup a dma from/to the external PIB master.

    **Description**\n
    This function is used to do DMA based transfer from/to the external PIB master.
    This function is a blocking call which waits until the desired transfer is
    complete or the specified timeout duration has elapsed.

    It is expected that the length of data being transferred is a multiple of
    16 bytes.

    **Return value**\n
    * CY_FX3_BOOT_SUCCESS                - if the data transfer is successful.\n
    * CY_FX3_BOOT_ERROR_INVALID_DMA_ADDR - if the address specified is not in the SYSMEM area.\n
    * CY_FX3_BOOT_ERROR_XFER_FAILURE     - if the data transfer encountered any error.\n
    * CY_FX3_BOOT_ERROR_TIMEOUT          - if the data transfer times out.
 */
extern CyFx3BootErrorCode_t
CyFx3BootPibDmaXferData (
        uint8_t  sockNum,       /**< Socket through which to do the data transfer. */
        CyBool_t isRead,        /**< isRead=CyTrue for read transfers, and isRead=CyFalse for write transfers. */
        uint32_t address,       /**< Address of the buffer from/to which data is to be transferred */
        uint32_t length,        /**< Length of the data to be transferred. Should be a multiple of
                                     16 bytes. */
        uint32_t timeout        /**< Timeout duration in multiples of 10 us. Can be set to CY_FX3_BOOT_WAIT_FOREVER
                                     to wait until the transfer is complete. */
        );

/** \brief Register a callback function to be called on GPIF interrupt.

    **Description**\n
    This function registers a callback function that will be called when the GPIF state machine
    triggers an interrupt. No other GPIF interrupt types are supported.

    **Return value**\n
    * None

    **\see
    *\see CyFx3BootGpifIntrCb_t
 */
extern void
CyFx3BootGpifRegisterCallback (
        CyFx3BootGpifIntrCb_t cbFunc    /**< Callback function pointer. */
        );

/** \brief Configure the GPIF II hardware functionality.

    **Description**\n
    This function configures the GPIF II hardware on the FX3 device to implement the desired
    interface and functionality. The configuration data passed as an argument to this function
    is generated by the GPIF II Designer tool.

    **Return value**\n
    * CY_FX3_BOOT_SUCCESS - if the configuration load is successful.\n
    * CY_FX3_BOOT_ERROR_NOT_SUPPORTED - if the configuration being loaded cannot be supported.\n
    * CY_FX3_BOOT_ERROR_ALREADY_STARTED - if the GPIF hardware is already configured and running.\n
    * CY_FX3_BOOT_ERROR_BAD_ARGUMENT - if the input structure is inconsistent.

    **\see
    *\see CyU3PGpifConfig_t
 */
extern CyFx3BootErrorCode_t
CyFx3BootGpifLoad (
        const CyU3PGpifConfig_t *conf           /**< Pointer to GPIF configuration information. */
        );

/** \brief Disable the GPIF II State Machine and Hardware.

    **Description**\n
    This function disables the GPIF II State Machine and hardware. If the forceReload parameter
    is true, the current configuration is cleared completely, and the CyFx3BootGpifLoad
    function must be called again. If the forceReload parameter is not set, the GPIF configuration
    is retained in memory, and the state machine can be restarted using the CyFx3BootGpifSMStart
    API.

    **Return value**\n
    * None

    **\see
    *\see CyFx3BootGpifLoad
    *\see CyFx3BootGpifSMStart
 */
extern void
CyFx3BootGpifDisable (
        CyBool_t forceReload                    /**< Whether a GPIF re-configuration is to be forced. */
        );

/** \brief Start the GPIF II State Machine from the specified state.

    **Description**\n
    This function starts the GPIF II State Machine from the specified state index. This can
    only be used to start the GPIF II State Machine from an idle state. The CyFx3BootGpifSMSwitch
    API should be used to switch the state machine from one state to another in mid-execution.

    **Return value**\n
    * CY_FX3_BOOT_SUCCESS - if the state machine is started successfully.\n
    * CY_FX3_BOOT_ERROR_NOT_CONFIGURED - if the GPIF hardware has not been configured.\n
    * CY_FX3_BOOT_ERROR_ALREADY_STARTED - if the state machine has already been started.

    **\see
    *\see CyFx3BootGpifSMSwitch
 */
extern CyFx3BootErrorCode_t
CyFx3BootGpifSMStart (
        uint8_t  startState,            /**< State from which to start state machine execution. This should
                                             be one of the start states in the GPIF state machine design. */
        uint8_t  initialAlpha           /**< Initial values of alpha inputs to start the GPIF state machine
                                             with. */
        );

/** \brief This function is used to start GPIF state machine execution from a desired
    state.

    **Description**\n
    This function allows the caller to switch to a desired state and continue
    GPIF state machine execution from there. The toState parameter specifies
    the state to initiate operation from.

    The fromState parameter can be used to ensure that the transition to toState
    happens only when the state machine is in a well defined idle state. If a
    valid state id (0 - 255) is passed as the fromState, the transition is only
    allowed from that state index. If not, the state machine is immediately
    switched to the toState.

    **Return value**\n
    * CY_FX3_BOOT_SUCCESS - if the switch request is successful.\n
    * CY_FX3_BOOT_ERROR_BAD_ARGUMENT - if the parameters are invalid.

    **\see
    *\see CyFx3BootGpifSMStart
 */
extern CyFx3BootErrorCode_t
CyFx3BootGpifSMSwitch (
        uint16_t fromState,             /**< State from which to do the switch. Can be set to an invalid value
                                             to force an immediate switch. */
        uint16_t toState,               /**< State to switch operation into. Must be a valid state number. */
        uint8_t  initialAlpha           /**< Initial values for the alpha inputs when switching states. */
        );

/** \brief Function to initialize one of the GPIF II Counters.

    **Description**\n
    This function initializes one of the GPIF II counters with the specified parameters.

    **Return value**\n
    * CY_FX3_BOOT_SUCCESS - if the counter is configured properly.\n
    * CY_FX3_BOOT_ERROR_NOT_STARTED - if the PIB block is not started.\n
    * CY_FX3_BOOT_ERROR_NOT_CONFIGURED - if the PIB block is configured in MMC mode.\n
    * CY_FX3_BOOT_ERROR_BAD_ARGUMENT - if the parameters are incorrect.

    **\see
    *\see CyFx3GpifCounterType
 */
extern CyFx3BootErrorCode_t
CyFx3BootGpifInitCounter (
        CyFx3GpifCounterType counter,   /**< Type of counter to be initialized. */
        uint32_t             initValue, /**< Initial value for the counter. */
        uint32_t             limit,     /**< Value at which the counter must be stopped. */
        CyBool_t             reload,    /**< Whether the counter should be reloaded and restarted after
                                             reaching the limit. */
        int8_t               increment, /**< Value to update the counter by at each step. Can be negative. */
        uint8_t              outputbit  /**< Selects the control counter bit to be connected to the CTRL[9] output. */
        );

/** \brief Change the state of the firmware input to the GPIF II State Machine.

    **Description**\n
    The GPIF II State Machine is capable of waiting for firmware readiness using an input
    signal that can be controlled by the firmware. This function updates the state of this
    input signal as desired.

    **Return value**\n
    * None
 */
extern void
CyFx3BootGpifControlSWInput (
        CyBool_t set                    /**< Whether to set (CyTrue) or clear (CyFalse) the input signal. */
        );

/** \brief Get the current GPIF state.

    **Description**\n
    This function gets the current GPIF II state machine state.

    **Return value**\n
    * The current state. 0 if the state cannot be determined.
 */
extern uint8_t
CyFx3BootGpifGetState (
        void);

/** \brief Function to select an active socket on the P-port and to configure it.

    **Description**\n
    The GPIF hardware allows 4 different sockets on the P-port to be accessed at a time
    by supporting 4 independent DMA threads. The active socket for each thread and its
    properties can be selected by the user at run-time. This should be done in software
    only in the case where it is not being done through the PP registers or the state
    machine itself.

    This function allows the user to select and configure the active socket in the case
    where software is responsible for these actions. The API will respond with an error
    if the hardware is taking care of socket configurations.

    **Return value**\n
    * CY_FX3_BOOT_SUCCESS - if successful.\n
    * CY_FX3_BOOT_ERROR_FAILURE - if the socket selection and configuration is being done automatically.\n
    * CY_FX3_BOOT_ERROR_BAD_ARGUMENT - if one or more of the parameters are out of range.
 */
extern CyFx3BootErrorCode_t
CyFx3BootGpifSocketConfigure (
        uint8_t  threadIndex,   /**< Thread index whose active socket is to be configured. */
        uint8_t  socketNum,     /**< The socket to be associated with this thread. */
        uint16_t watermark,     /**< Watermark level for this socket in number of 4-byte words. */
        CyBool_t flagOnData,    /**< Whether the partial flag should be set when the socket contains
                                     more data than the watermark. If false, the flag will be set when
                                     the socket contains less data than the watermark. */
        uint8_t  burst          /**< Logarithm (to the base 2) of the burst size for this socket.
                                     The burst size is the minimum number of words of data that will
                                     be sourced/sinked across the GPIF interface without further
                                     updates of the GPIF DMA flags. The device connected to FX3 is
                                     expected to complete a burst that it has started regardless of any
                                     flag changes in between. Please note that this has to be set to
                                     a non-zero value (burst size is greater than one), when the GPIF
                                     is being configured with a 32-bit data bus and functioning at
                                     100 MHz. */
        );

#include <cyu3externcend.h>

#endif /* _INCLUDED_CYFX3PIB_H_ */

/*[]*/
