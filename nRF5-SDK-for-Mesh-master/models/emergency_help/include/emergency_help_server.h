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

#ifndef EMERGENCY_HELP_SERVER_H__
#define EMERGENCY_HELP_SERVER_H__

#include <stdint.h>
#include <stdbool.h>
#include "access.h"

/**
 * @defgroup SIMPLE_ON_OFF_SERVER Simple OnOff Server
 * @ingroup SIMPLE_ON_OFF_MODEL
 * This module implements a vendor specific Simple OnOff Server.
 * @{
 */

/** Simple OnOff Server model ID. */
#define EMERGENCY_HELP_SERVER_MODEL_ID (0x0004)

/** Forward declaration. */
typedef struct __emergency_help_server emergency_help_server_t;

/**
 * Set callback type.
 * @param[in] p_self Pointer to the Simple OnOff Server context structure.
 * @param[in] on_off Desired state
 * @returns @c true if the current state is On, @c false otherwise.
 */
typedef bool (*emergency_help_set_cb_t)(const emergency_help_server_t * p_self, uint16_t en_emh, uint16_t src);

typedef void (*emergency_help_set_unreliable_cb_t)(const emergency_help_server_t *p_self, uint16_t src);

/** Simple OnOff Server state structure. */
struct __emergency_help_server
{
    /** Model handle assigned to the server. */
    access_model_handle_t model_handle;

    /** Set callback. */
    emergency_help_set_cb_t set_cb;

    emergency_help_set_unreliable_cb_t set_unreliable_cb;
};

/**
 * Initializes the Simple OnOff server.
 *
 * @note This function should only be called _once_.
 * @note The server handles the model allocation and adding.
 *
 * @param[in] p_server      Simple OnOff Server structure pointer.
 * @param[in] element_index Element index to add the server model.
 *
 * @retval NRF_SUCCESS         Successfully added server.
 * @retval NRF_ERROR_NULL      NULL pointer supplied to function.
 * @retval NRF_ERROR_NO_MEM    No more memory available to allocate model.
 * @retval NRF_ERROR_FORBIDDEN Multiple model instances per element is not allowed.
 * @retval NRF_ERROR_NOT_FOUND Invalid element index.
 */
uint32_t emergency_help_server_init(emergency_help_server_t * p_server, uint16_t element_index);

/** @} end of emergency_help_SERVER */

#endif /* SIMPLE_ON_OFF_SERVER_H__ */
