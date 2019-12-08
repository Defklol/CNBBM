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

#include "simple_battery_server.h"
#include "simple_battery_common.h"

#include <stdint.h>
#include <stddef.h>

#include "access.h"
#include "nrf_mesh_assert.h"
#include "nrf_drv_saadc.h"
#include "log.h"

static nrf_saadc_value_t      adc_value;

/*****************************************************************************
 * Static functions
 *****************************************************************************/


/*****************************************************************************
 * Opcode handler callbacks
 *****************************************************************************/
static void handle_set_unreliable_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    uint32_t err_code;

    //simple_battery_rcv_set_unreliable_t rcv_set_unreliable;
    simple_battery_server_t * p_server = p_args;

    //rcv_set_unreliable.battery = *(p_message->p_data);
    //rcv_set_unreliable.tid = *(p_message->p_data+1);
    //rcv_set_unreliable.device_ID = *(p_message->p_data+2);
    
    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Receive ADC Address %u , Tid Address %u, Device ID Address %u\n", p_message->p_data, p_message->p_data+1, p_message->p_data+2);
    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Receive ADC Value %d , Tid %d, Device ID %04x, SRC 0x%04x\n", rcv_set_unreliable.battery, rcv_set_unreliable.tid, rcv_set_unreliable.device_ID, p_message->meta_data.src.value);
    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "0:%x 1:%x 2:%x 3:%x 4:%x\n", *(p_message->p_data), *(p_message->p_data+1), *(p_message->p_data+2), *(p_message->p_data+3));
    uint16_t src = *(p_message->p_data+3)<<8 |*(p_message->p_data+2);
    
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Receive BAT UNReliable MSG, BAT Value : %d, Tid %d, SRC 0x%04x\n", *(p_message->p_data), *(p_message->p_data+1), src);

    p_server->set_unreliable_cb( p_server, *(p_message->p_data), src);
}

static const access_opcode_handler_t m_opcode_handlers[] =
{
    {ACCESS_OPCODE_VENDOR(SIMPLE_BATTERY_OPCODE_SET_UNRELIABLE, SIMPLE_BATTERY_COMPANY_ID), handle_set_unreliable_cb}
};

/*****************************************************************************
 * Public API
 *****************************************************************************/

uint32_t simple_battery_server_init(simple_battery_server_t * p_server, uint16_t element_index)
{
    if (p_server == NULL)
    {
        return NRF_ERROR_NULL;
    }

    access_model_add_params_t init_params;
    init_params.element_index =  element_index;
    init_params.model_id.model_id = SIMPLE_BATTERY_SERVER_MODEL_ID;
    init_params.model_id.company_id = SIMPLE_BATTERY_COMPANY_ID;
    init_params.p_opcode_handlers = &m_opcode_handlers[0];
    init_params.opcode_count = sizeof(m_opcode_handlers) / sizeof(m_opcode_handlers[0]);
    init_params.p_args = p_server;
    init_params.publish_timeout_cb = NULL;
    return access_model_add(&init_params, &p_server->model_handle);
}
