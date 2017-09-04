/*
 ## Cypress FX3 Boot Firmware Header (cyfx3dma.h)
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

#ifndef _INCLUDED_CYFX3DMA_H_
#define _INCLUDED_CYFX3DMA_H_

#include <cyfx3error.h>
#include <cyu3types.h>
#include <cyfx3device.h>
#include <cyu3externcstart.h>

/** \file cyfx3dma.h
    \brief The DMA driver module in the FX3 boot firmware provides a set of
    APIs to manage data transfers, and provides DMA interrupt handlers.
 */

/**< Number of USB Ingress Sockets. */
#define CY_FX3_DMA_USB_OUT_SOCKCNT      (16)

/**< Number of USB Egress Sockets. */
#define CY_FX3_DMA_USB_IN_SOCKCNT       (16)

/**< Number of PIB Sockets. */
#define CY_FX3_DMA_PIB_SOCKCNT          (32)

/**< Number of LPP Sockets. */
#define CY_FX3_DMA_LPP_SOCKCNT          (8)

/**< DMA descriptor used for USB transfers. */
#define CY_FX3_USB_DMA_DSCR_INDEX       (1)

/**< DMA descriptor used for LPP transfers. */
#define CY_FX3_LPP_DMA_DSCR_INDEX       (2)

/**< DMA descriptor used for PIB transfers. */
#define CY_FX3_PIB_DMA_DSCR_INDEX       (3)

/**< Minimum allowed descriptor index. The first 4 descriptors are reserved for API internal usage. */
#define CY_FX3_DMA_MIN_DSCR_INDEX       (4)

/**< Maximum allowed descriptor index. This is based on an allocation of 8 KB for DMA descriptors. */
#define CY_FX3_DMA_MAX_DSCR_INDEX       (511)

/**< Calculate DMA descriptor address corresponding to an index. */
#define CY_FX3_DMA_GETDSCR_BY_INDEX(i)  ((CyFx3BootDmaDescriptor_t *)(0x40000000 + ((i) << 4)))

/** \brief DMA socket IDs for all sockets in the device.

    **Description**\n
    This is a software representation of all sockets on the device.
    The socket ID has two parts: IP number and socket number. Each
    peripheral (IP) has a fixed ID. LPP is 0, PIB is 1, Storage port
    is 2, USB egress is 3 and USB ingress is 4.
    
    Each peripheral has a number of sockets. The LPP sockets are fixed
    and have to be used as defined. The PIB sockets 0-15 can be used as
    both producer and consumer, but the PIB sockets 16-31 are strictly
    producer sockets. The UIB sockets are defined as 0-15 producer and
    0-15 consumer sockets. The CPU sockets are virtual representations
    and have no backing hardware block.
 */
typedef enum CyFx3BootDmaSockId_t
{
    CY_DMA_LPP_SOCKET_I2S_LEFT = 0x0000,        /**< Left channel output to I2S port. */
    CY_DMA_LPP_SOCKET_I2S_RIGHT,                /**< Right channel output to I2S port. */
    CY_DMA_LPP_SOCKET_I2C_CONS,                 /**< Outgoing data to I2C slave. */
    CY_DMA_LPP_SOCKET_UART_CONS,                /**< Outgoing data to UART peer. */
    CY_DMA_LPP_SOCKET_SPI_CONS,                 /**< Outgoing data to SPI slave. */
    CY_DMA_LPP_SOCKET_I2C_PROD,                 /**< Incoming data from I2C slave. */
    CY_DMA_LPP_SOCKET_UART_PROD,                /**< Incoming data from UART peer. */
    CY_DMA_LPP_SOCKET_SPI_PROD,                 /**< Incoming data from SPI slave. */


    CY_DMA_PIB_SOCKET_0 = 0x0100,               /**< P-port socket number 0. */
    CY_DMA_PIB_SOCKET_1,                        /**< P-port socket number 1. */
    CY_DMA_PIB_SOCKET_2,                        /**< P-port socket number 2. */
    CY_DMA_PIB_SOCKET_3,                        /**< P-port socket number 3. */
    CY_DMA_PIB_SOCKET_4,                        /**< P-port socket number 4. */
    CY_DMA_PIB_SOCKET_5,                        /**< P-port socket number 5. */
    CY_DMA_PIB_SOCKET_6,                        /**< P-port socket number 6. */
    CY_DMA_PIB_SOCKET_7,                        /**< P-port socket number 7. */
    CY_DMA_PIB_SOCKET_8,                        /**< P-port socket number 8. */
    CY_DMA_PIB_SOCKET_9,                        /**< P-port socket number 9. */
    CY_DMA_PIB_SOCKET_10,                       /**< P-port socket number 10. */
    CY_DMA_PIB_SOCKET_11,                       /**< P-port socket number 11. */
    CY_DMA_PIB_SOCKET_12,                       /**< P-port socket number 12. */
    CY_DMA_PIB_SOCKET_13,                       /**< P-port socket number 13. */
    CY_DMA_PIB_SOCKET_14,                       /**< P-port socket number 14. */
    CY_DMA_PIB_SOCKET_15,                       /**< P-port socket number 15. */
    CY_DMA_PIB_SOCKET_16,                       /**< P-port socket number 16. */
    CY_DMA_PIB_SOCKET_17,                       /**< P-port socket number 17. */
    CY_DMA_PIB_SOCKET_18,                       /**< P-port socket number 18. */
    CY_DMA_PIB_SOCKET_19,                       /**< P-port socket number 19. */
    CY_DMA_PIB_SOCKET_20,                       /**< P-port socket number 20. */
    CY_DMA_PIB_SOCKET_21,                       /**< P-port socket number 21. */
    CY_DMA_PIB_SOCKET_22,                       /**< P-port socket number 22. */
    CY_DMA_PIB_SOCKET_23,                       /**< P-port socket number 23. */
    CY_DMA_PIB_SOCKET_24,                       /**< P-port socket number 24. */
    CY_DMA_PIB_SOCKET_25,                       /**< P-port socket number 25. */
    CY_DMA_PIB_SOCKET_26,                       /**< P-port socket number 26. */
    CY_DMA_PIB_SOCKET_27,                       /**< P-port socket number 27. */
    CY_DMA_PIB_SOCKET_28,                       /**< P-port socket number 28. */
    CY_DMA_PIB_SOCKET_29,                       /**< P-port socket number 29. */
    CY_DMA_PIB_SOCKET_30,                       /**< P-port socket number 30. */
    CY_DMA_PIB_SOCKET_31,                       /**< P-port socket number 31. */


    CY_DMA_SIB_SOCKET_0 = 0x0200,               /**< S-port socket number 0. */
    CY_DMA_SIB_SOCKET_1,                        /**< S-port socket number 1. */
    CY_DMA_SIB_SOCKET_2,                        /**< S-port socket number 2. */
    CY_DMA_SIB_SOCKET_3,                        /**< S-port socket number 3. */
    CY_DMA_SIB_SOCKET_4,                        /**< S-port socket number 4. */
    CY_DMA_SIB_SOCKET_5,                        /**< S-port socket number 5. */


    CY_DMA_UIB_SOCKET_CONS_0 = 0x0300,          /**< U-port output socket number 0. */
    CY_DMA_UIB_SOCKET_CONS_1,                   /**< U-port output socket number 1. */
    CY_DMA_UIB_SOCKET_CONS_2,                   /**< U-port output socket number 2. */
    CY_DMA_UIB_SOCKET_CONS_3,                   /**< U-port output socket number 3. */
    CY_DMA_UIB_SOCKET_CONS_4,                   /**< U-port output socket number 4. */
    CY_DMA_UIB_SOCKET_CONS_5,                   /**< U-port output socket number 5. */
    CY_DMA_UIB_SOCKET_CONS_6,                   /**< U-port output socket number 6. */
    CY_DMA_UIB_SOCKET_CONS_7,                   /**< U-port output socket number 7. */
    CY_DMA_UIB_SOCKET_CONS_8,                   /**< U-port output socket number 8. */
    CY_DMA_UIB_SOCKET_CONS_9,                   /**< U-port output socket number 9. */
    CY_DMA_UIB_SOCKET_CONS_10,                  /**< U-port output socket number 10. */
    CY_DMA_UIB_SOCKET_CONS_11,                  /**< U-port output socket number 11. */
    CY_DMA_UIB_SOCKET_CONS_12,                  /**< U-port output socket number 12. */
    CY_DMA_UIB_SOCKET_CONS_13,                  /**< U-port output socket number 13. */
    CY_DMA_UIB_SOCKET_CONS_14,                  /**< U-port output socket number 14. */
    CY_DMA_UIB_SOCKET_CONS_15,                  /**< U-port output socket number 15. */


    CY_DMA_UIB_SOCKET_PROD_0 = 0x400,           /**< U-port input socket number 0. */
    CY_DMA_UIB_SOCKET_PROD_1,                   /**< U-port input socket number 1. */
    CY_DMA_UIB_SOCKET_PROD_2,                   /**< U-port input socket number 2. */
    CY_DMA_UIB_SOCKET_PROD_3,                   /**< U-port input socket number 3. */
    CY_DMA_UIB_SOCKET_PROD_4,                   /**< U-port input socket number 4. */
    CY_DMA_UIB_SOCKET_PROD_5,                   /**< U-port input socket number 5. */
    CY_DMA_UIB_SOCKET_PROD_6,                   /**< U-port input socket number 6. */
    CY_DMA_UIB_SOCKET_PROD_7,                   /**< U-port input socket number 7. */
    CY_DMA_UIB_SOCKET_PROD_8,                   /**< U-port input socket number 8. */
    CY_DMA_UIB_SOCKET_PROD_9,                   /**< U-port input socket number 9. */
    CY_DMA_UIB_SOCKET_PROD_10,                  /**< U-port input socket number 10. */
    CY_DMA_UIB_SOCKET_PROD_11,                  /**< U-port input socket number 11. */
    CY_DMA_UIB_SOCKET_PROD_12,                  /**< U-port input socket number 12. */
    CY_DMA_UIB_SOCKET_PROD_13,                  /**< U-port input socket number 13. */
    CY_DMA_UIB_SOCKET_PROD_14,                  /**< U-port input socket number 14. */
    CY_DMA_UIB_SOCKET_PROD_15,                  /**< U-port input socket number 15. */


    CY_DMA_CPU_SOCKET_CONS = 0x3F00,            /**< Socket through which the FX3 CPU receives data. */
    CY_DMA_CPU_SOCKET_PROD                      /**< Socket through which the FX3 CPU produces data. */

} CyFx3BootDmaSockId_t;

/** \brief Descriptor data structure.

    **Description**\n
    This data structure contains the fields that make up a DMA descriptor on
    the FX3 device.
 */
typedef struct CyFx3BootDmaDescriptor_t
{
    uint8_t    *buffer;    /**< Pointer to buffer used. */
    uint32_t    sync;      /**< Consumer, Producer binding. */
    uint32_t    chain;     /**< Next descriptor links. */
    uint32_t    size;      /**< Current and maximum sizes of buffer. */
} CyFx3BootDmaDescriptor_t;

/** \brief DMA socket configuration structure.

    **Description**\n
    This structure holds all the configuration fields that can be directly updated
    on a DMA socket. Refer to the sock_regs.h file for the detailed break up into
    bit-fields for each of the structure members.
 */
typedef struct CyFx3BootDmaSocket_t
{
    uint32_t dscrChain;         /**< The descriptor chain associated with the socket. */
    uint32_t xferSize;          /**< Transfer size for the socket. */
    uint32_t xferCount;         /**< Transfer status for the socket. */
    uint32_t status;            /**< Socket status register. */
    uint32_t intr;              /**< Interrupt status. */
    uint32_t intrMask;          /**< Interrupt mask. */
} CyFx3BootDmaSocket_t;

/** \brief Actual socket specific register structure.

    **Description**\n
    This structure represents the actual socket status register space on the FX3 device.
    This includes a number of reserved and hardware-access only registers which are skipped
    in the CyFx3BootDmaSocket_t structure.
 */
typedef struct CyFx3BootDmaSockRegs_t
{
    uvint32_t dscrChain;                /**< The descriptor chain associated with the socket */
    uvint32_t xferSize;                 /**< The transfer size requested for this socket. The size can
                                             be specified in bytes or in terms of number of buffers,
                                             depending on the UNIT field in the status value. */
    uvint32_t xferCount;                /**< The completed transfer count for this socket. */
    uvint32_t status;                   /**< Socket configuration and status register. */
    uvint32_t intr;                     /**< Interrupt status register. */
    uvint32_t intrMask;                 /**< Interrupt mask register. */
    uint32_t unused2[2];                /**< Reserved register space. */
    uvint32_t actBuffer;                /**< Buffer pointer from current DMA descriptor. */
    uvint32_t actSync;                  /**< Sync field from current DMA descriptor. */
    uvint32_t actChain;                 /**< Chain field from current DMA descriptor. */
    uvint32_t actSize;                  /**< Size field from current DMA descriptor. */
    uint32_t unused19[19];              /**< Reserved register space. */
    uvint32_t sckEvent;                 /**< Generate event register. */
} CyFx3BootDmaSockRegs_t;

/** \brief DMA socket interrupt handler callback function.

    **Description**\n
    Callback function type used to notify user about DMA socket interrupts. This callback
    will be invoked from interrupt context with all other interrupts disabled. Adding
    delays in the callback implementation may adversely impact handling of USB block
    interrupts.

    **\see
    *\see CyFx3BootDmaRegisterCallback
 */
typedef void (*CyFx3BootDmaCallback_t) (
        uint16_t sckId,         /**< ID of the socket triggering the interrupt. */
        uint32_t status         /**< Socket interrupt status. */
        );

/** \brief Register a callback for notification of DMA interrupts.

    **Description**\n
    Register a callback function for notification of DMA interrupts from various FX3 blocks.
    The callback function pointer a set of block level interrupt enable flags are provided
    as parameters. The driver will only enable interrupts at the VIC level for blocks that
    have their interrupt enable flags set.

    **Note**\n
    The callback is responsible for clearing the interrupt. The ISR does not clear any interrupts,
    as this may cause some interrupts to get lost.

    **\see
    *\see CyFx3BootDmaCallback_t
 */
extern void
CyFx3BootDmaRegisterCallback (
        CyFx3BootDmaCallback_t cbFunc,                  /**< Callback function pointer. */
        CyBool_t               usbIntrEn,               /**< Enable DMA interrupt for USB endpoints. */
        CyBool_t               pibIntrEn,               /**< Enable DMA interrupt for the GPIF-II / PMMC block. */
        CyBool_t               serialIntrEn             /**< Enable DMA interrupt for the serial (SPI, I2C, I2S and
                                                             UART) interface blocks. */
        );

/** \brief Get the current values from the specified DMA descriptor index.

    **Description**\n
    A DMA descriptor structure maintains information about a single DMA buffer on the FX3 device.
    An area of the FX3 device SRAM is reserved for DMA descriptor usage, and the address for each
    descriptor is a function of the descriptor index. This function fetches the current values
    from the specified descriptor index. This is typically called as part of a read-modify-write
    sequence to setup the descriptors.

    **Note**\n
    Descriptor indices 0-3 are reserved and should not be used. The valid range is from 4 to 511.

    **Returns**\n
    * CY_FX3_BOOT_SUCCESS if the descriptor fetch is successful.\n
    * CY_FX3_BOOT_ERROR_BAD_ARGUMENT if the descriptor index or dscr_p parameter is invalid.

    **\see
    *\see CyFx3BootDmaDescriptor_t
    *\see CyFx3BootDmaSetDscrConfig
 */
extern CyFx3BootErrorCode_t
CyFx3BootDmaGetDscrConfig (
        uint16_t                  dscrIndex,            /**< Index of descriptor to be updated. */
        CyFx3BootDmaDescriptor_t *dscr_p                /**< Structure to be filled with descriptor configuration. */
        );

/** \brief Update the values of a DMA descriptor on the FX3 device.

    **Description**\n
    This function updates the values of a DMA descriptor from the contents of the structure parameter
    passed. This is typically called as part of a descriptor read-modify-write sequence.

    **Note**\n
    Descriptor indices 0-3 are reserved and should not be used. The valid range is from 4 to 511.

    **Returns**\n
    * CY_FX3_BOOT_SUCCESS if the descriptor fetch is successful.\n
    * CY_FX3_BOOT_ERROR_BAD_ARGUMENT if the descriptor index or dscr_p parameter is invalid.

    **\see
    *\see CyFx3BootDmaDescriptor_t
    *\see CyFx3BootDmaGetDscrConfig
 */
extern CyFx3BootErrorCode_t
CyFx3BootDmaSetDscrConfig (
        uint16_t                  dscrIndex,            /**< Index of descriptor to be updated. */
        CyFx3BootDmaDescriptor_t *dscr_p                /**< Structure containing new descriptor configuration. */
        );

/** \brief Get the current interrupt status for a given socket.

    **Description**\n
    This function gets the current interrupt status value for a given socket.

    **Returns**\n
    * Current interrupt status value. Will be 0 if the socket is invalid.
 */
extern uint32_t
CyFx3BootDmaGetSockInterrupts (
        CyFx3BootDmaSockId_t sockId                     /**< Socket on which interrupts are to be queried. */
        );

/** \brief Get the current status of a DMA socket.

    **Description**\n
    A DMA socket is a construct that manages a unique data pipe between FX3 and an external device.
    All configuration and status information about a socket are stored in the form of a set of
    registers, and can be queried using this API. This API is typically called as part of a 
    read-modify-write sequence to update the socket configuration.

    **Returns**\n
    * CY_FX3_BOOT_SUCCESS if the socket fetch is successful.\n
    * CY_FX3_BOOT_ERROR_BAD_ARGUMENT if the socket id or sock_p parameter is invalid.

    **\see
    *\see CyFx3BootDmaSockId_t
    *\see CyFx3BootDmaSocket_t
    *\see CyFx3BootDmaSetSocketConfig
 */
extern CyFx3BootErrorCode_t
CyFx3BootDmaGetSocketConfig (
        CyFx3BootDmaSockId_t  sockId,                   /**< Id of socket to be retrieved. */
        CyFx3BootDmaSocket_t *sock_p                    /**< Structure into which the configuration is queried. */
        );

/** \brief Update the configuration of a DMA socket.

    **Description**\n
    This API updates the configuration of a DMA socket based on the contents of the structure
    parameter. This is typically called as part of a read-modify-write sequence to update the
    socket configuration. It is also possible to set the ENABLE bit in the socket as part of this
    API and avoid calling CyFx3BootDmaEnableSocket.

    **Note**\n
    Please note that the sock_p->intr value is a write one to clear setting. Calling this API
    with a non-zero interrupt intr value will cause some interrupts to (unexpectedly) be cleared.

    **Returns**\n
    * CY_FX3_BOOT_SUCCESS if the socket update is successful.\n
    * CY_FX3_BOOT_ERROR_BAD_ARGUMENT if the socket id or sock_p parameter is invalid.\n
    * CY_FX3_BOOT_ERROR_ALREADY_STARTED if a socket is being updated while it is in the active state.

    **\see
    *\see CyFx3BootDmaSockId_t
    *\see CyFx3BootDmaSocket_t
    *\see CyFx3BootDmaGetSocketConfig
 */
extern CyFx3BootErrorCode_t
CyFx3BootDmaSetSocketConfig (
        CyFx3BootDmaSockId_t  sockId,                   /**< Id of socket to be updated. */
        CyFx3BootDmaSocket_t *sock_p                    /**< Structure containing new socket configuration. */
        );

/** \brief Forcibly commit the currently available data in an ingress socket.

    **Description**\n
    Data received into an ingress (data incoming) socket on FX3 will only be committed (made available
    to CPU or for sending out from the device) when the buffer has been filled, or when a end of packet
    (buffer) signal is received. In some cases, neither of these events may happen for a long time;
    leaving a buffer partially filled with data. This API can be used to forcibly wrap-up (commit)
    the available data on an ingress socket.

    **Returns**\n
    * CY_FX3_BOOT_SUCCESS if the socket wrap-up is successful.\n
    * CY_FX3_BOOT_ERROR_BAD_ARGUMENT if the sockId parameter is invalid.
 */
extern CyFx3BootErrorCode_t
CyFx3BootDmaWrapSocket (
        CyFx3BootDmaSockId_t sockId                     /**< Id of socket to be wrapped up. */
        );

/** \brief Disable a DMA socket.

    **Description**\n
    This API forcibly disables the specified DMA socket. This can be used to ensure that all sockets
    start from a clean state, or as part of error handling.

    **Returns**\n
    * CY_FX3_BOOT_SUCCESS if the socket is disabled.
    * CY_FX3_BOOT_ERROR_BAD_ARGUMENT if the socket id is invalid.

    **\see
    *\see CyFx3BootDmaEnableSocket
 */
extern CyFx3BootErrorCode_t
CyFx3BootDmaDisableSocket (
        CyFx3BootDmaSockId_t sockId                     /**< Id of socket to be disabled. */
        );

/** \brief Enable a DMA socket.

    **Description**\n
    Enables the specified DMA socket for operation. Data transfers through the socket can happen
    once it is enabled. The typical sequence for starting up a DMA transfer is to configure the
    sockets using CyFx3BootDmaGetSocketConfig and CyFx3BootDmaSetSocketConfig, and then to enable
    the sockets using CyFx3BootDmaEnableSocket.

    **Returns**\n
    * CY_FX3_BOOT_SUCCESS if the socket was enabled.
    * CY_FX3_BOOT_ERROR_BAD_ARGUMENT if the socket id is invalid.
    * CY_FX3_BOOT_ERROR_ALREADY_STARTED if the socket is already enabled.

    **\see
    *\see CyFx3BootDmaDisableSocket
 */
extern CyFx3BootErrorCode_t
CyFx3BootDmaEnableSocket (
        CyFx3BootDmaSockId_t sockId                     /**< Id of socket to be enabled. */
        );

/** \brief Send an event to notify a socket that a desciptor that is waiting on has been modified.

     **Description**\n
     A data path through the FX3 device involves a pair of ingress (producer) and egress (consumer)
     sockets that are bound together with a set of DMA descriptors and buffers. The two sockets at
     the ends of the data path communicate through the shared descriptor structures which are maintained
     in memory. Once a socket has updated a DMA descriptor in memory, it notifies its peer socket
     by writing into its EVENT register.

     In cases where the communication between the two sockets is being controlled by firmware
     (manual DMA channels), the sockets are not configured to send these EVENTS directly to the
     peer socket. Instead, the CPU gets notified through an interrupt; and then updates the socket
     by writing to the EVENT register using this API.

     **Returns**\n
     * CY_FX3_BOOT_SUCCESS if the event sending is successful.\n
     * CY_FX3_BOOT_ERROR_BAD_ARGUMENT if the parameters are invalid.
 */
extern CyFx3BootErrorCode_t
CyFx3BootDmaSendSocketEvent (
        CyFx3BootDmaSockId_t sockId,                    /**< Socket to which the event has to be sent. */
        uint16_t             dscrIndex,                 /**< Index of descriptor that has been updated. */
        CyBool_t             isOccupied                 /**< Whether the descriptor has been marked as occupied (=1)
                                                             or empty (=0). */
        );

/** \brief Clear the specified socket interrupts.

    **Description**\n
    This function clears the specified interrupt on a DMA socket.
 */
extern void
CyFx3BootDmaClearSockInterrupts (
        CyFx3BootDmaSockId_t sockId,                    /**< Socket on which interrupts are to be cleared. */
        uint32_t             intrVal                    /**< Bitmask representing interrupts to be cleared. */
        );

#include <cyu3externcend.h>

#endif /* _INCLUDED_CYFX3DMA_H_ */

/*[]*/

