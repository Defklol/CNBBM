/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef EMERGENCY_HELP_CLIENT_H__
#define EMERGENCY_HELP_CLIENT_H__

#include <stdint.h>
#include "access.h"
#include "emergency_help_common.h"

/**
 * @defgroup SIMPLE_ON_OFF_CLIENT Simple OnOff Client
 * @ingroup SIMPLE_ON_OFF_MODEL
 * This module implements a vendor specific Simple OnOff Client.
 *
 * @{
 */


/** Simple OnOff Client model ID. */
#define EMERGENCY_HELP_CLIENT_MODEL_ID (0x0003)

/** Simple OnOff status codes. */
typedef enum
{
    /** Received status OFF from the server. */
    EMERGENCY_HELP_STATUS_RECEIVE,
    /** The server did not reply to a Simple OnOff Set/Get. */
    EMERGENCY_HELP_STATUS_ERROR_NO_REPLY,
    /** Simple OnOff Set/Get was cancelled. */
    EMERGENCY_HELP_STATUS_CANCELLED
} emergency_help_status_t;

/** Forward declaration. */
typedef struct __emergency_help_client emergency_help_client_t;

/**
 * Simple OnOff status callback type.
 *
 * @param[in] p_self Pointer to the Simple OnOff client structure that received the status.
 * @param[in] status The received status of the remote server.
 * @param[in] src    Element address of the remote server.
 */
typedef void (*emergency_help_status_cb_t)(const emergency_help_client_t * p_self, emergency_help_status_t status, uint16_t src);
/**
 * Simple OnOff timeout callback type.
 *
 * @param[in] handle Model handle
 * @param[in] p_self Pointer to the Simple OnOff client structure that received the status.
 */
typedef void (*emergency_help_timeout_cb_t)(access_model_handle_t handle, void * p_self);


/** Simple OnOff Client state structure. */
struct __emergency_help_client
{
    /** Model handle assigned to the client. */
    access_model_handle_t model_handle;
    /** Status callback called after status received from server. */
    emergency_help_status_cb_t status_cb;
    /** Timeout callback called after acknowledged message sending times out */
    emergency_help_timeout_cb_t timeout_cb;

    /** Internal client state. */
    struct
    {
        bool reliable_transfer_active; /**< Variable used to determine if a transfer is currently active. */
        emergency_help_msg_set_t data;  /**< Variable reflecting the data stored in the server. */
    } state;
};

/**
 * Initializes the Simple OnOff client.
 *
 * @note This function should only be called _once_.
 * @note The client handles the model allocation and adding.
 *
 * @param[in,out] p_client      Simple OnOff Client structure pointer.
 * @param[in]     element_index Element index to add the server model.
 *
 * @retval NRF_SUCCESS         Successfully added client.
 * @retval NRF_ERROR_NULL      NULL pointer supplied to function.
 * @retval NRF_ERROR_NO_MEM    No more memory available to allocate model.
 * @retval NRF_ERROR_FORBIDDEN Multiple model instances per element is not allowed.
 * @retval NRF_ERROR_NOT_FOUND Invalid element index.
 */
uint32_t emergency_help_client_init(emergency_help_client_t * p_client, uint16_t element_index);

/**
 * Sets the state of the Simple OnOff server.
 *
 * @param[in,out] p_client Simple OnOff Client structure pointer.
 * @param[in]     on_off   Value to set the Simple OnOff Server state to.
 *
 * @retval NRF_SUCCESS              Successfully sent message.
 * @retval NRF_ERROR_NULL           NULL pointer in function arguments
 * @retval NRF_ERROR_NO_MEM         Not enough memory available for message.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_ADDR   The element index is greater than the number of local unicast
 *                                  addresses stored by the @ref DEVICE_STATE_MANAGER.
 * @retval NRF_ERROR_INVALID_STATE  Message already scheduled for a reliable transfer.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to appkey, publish address not set or wrong
 *                                  opcode format.
 */
uint32_t emergency_help_client_set(emergency_help_client_t * p_client);

/**
 * Cancel any ongoing reliable message transfer.
 *
 * @param[in,out] p_client Pointer to the client instance structure.
 */
void emergency_help_client_pending_msg_cancel(emergency_help_client_t * p_client);

uint32_t emergency_help_client_set_unreliable(emergency_help_client_t * p_client, uint8_t repeats, uint16_t src);

/** @} end of SIMPLE_ON_OFF_CLIENT */

#endif /* SIMPLE_ON_OFF_CLIENT_H__ */
