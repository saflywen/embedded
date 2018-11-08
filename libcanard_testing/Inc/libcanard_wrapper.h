/**
 * @file libcanard_wrapper.h
 * Wrapper for libcanard UAVCAN library on STM32
 *
 *  Created on: Oct 28, 2018
 *      Author: David Lenfesty
 */

#ifndef LIBCANARD_WRAPPER_H_
#define LIBCANARD_WRAPPER_H_

#include "canard.h"

/* ------------ Error Definitions --------------- */

#define LIBCANARD_SUCCESS					1
#define LIBCANARD_NO_QUEUE					0
#define LIBCANARD_ERR						-1
#define LIBCANARD_ERR_NO_MEM				-2
#define LIBCANARD_ERR_INVALID_SETTINGS		-3
#define LIBCANARD_ERR_TX_QUEUE_FULL			-4
#define LIBCANARD_ERR_INVALID_ID			-5


/* ------------ Filtering Mask Definitions ------ */

#define CAN_MASK_UAVCAN_PRIORITY		(0b11111 << 24)
#define CAN_MASK_UAVCAN_MSG_TYPE		(0b1111111111111111 << 8)
#define CAN_MASK_UAVCAN_SRV_NOT_MSG		(0b1 << 7)
#define CAN_MASK_UAVCAN_SRC_NODE_ID		(0b1111111 << 0)

/* ------------ Value Definitions ---------------- */

// Memory pool size. Minimum is 1K.
#ifndef LIBCANARD_MEM_POOL_SIZE
#define LIBCANARD_MEM_POOL_SIZE 1024 // Default to 1K
#endif

#if LIBCANARD_MEM_POOL_SIZE < 1024
#error "Specified libcanard memory pool is too small!"
#endif


CanardInstance m_canard_instance;


/** @brief Initializes a libcanard instance
 *
 * 	@param on_reception Callback function called once a transfer is
 * 				fully received.
 *
 * 	@param should_accept Callback function to decide whether or not
 * 				to accept a transfer and process it further.
 *
 * 	@param user_reference Pointer passed around with libcanard instance.
 * 				Use this for keeping extra data around.
 *
 * 	@param node_id Node id you want to use. Can be a value from
 * 				0 -127.
 *
 * 	@param clock_rate Value (in Hz) of system clock.
 * 	@param bitrate Value (in bps) of bitrate you want to achieve.
 *
 * 	@returns Status of initialization functions.
 */
int16_t libcanard_init(	CanardOnTransferReception on_reception,
						CanardShouldAcceptTransfer should_accept,
						void* user_reference,
						uint8_t node_id,
						const uint32_t clock_rate,
						const uint32_t bitrate);



/** @brief Function to transmit a single frame from TX queue.
 *
 * 	@retval LIBCANARD_SUCCESS Successfully transmitted a single frame.
 * 	@retval LIBCANARD_ERR_TX_QUEUE_FULL Hardware TX queue is full.
 * 	@retval LIBCANARD_ERR Generic error.
 */
int8_t tx_once(void);



/** @brief Function to receive a single frame from the CAN hardware.
 *
 * 	@description Handles single reception from internal CAN hardware.
 * 			Will call both libcanard callbacks if a full transfer is received.
 *
 * 	@retval LIBCANARD_SUCCESS Frame successfully received.
 * 	@retval LIBCANARD_NO_QUEUE No RX queue to pull from.
 * 	@retval LIBCANARD_ERR Generic error.
 */
int8_t rx_once(void);


#endif /* LIBCANARD_WRAPPER_H_ */
