#include "simple_battery_client.h"
#include "simple_battery_common.h"

#include <stdint.h>
#include <stddef.h>

#include "access.h"
#include "access_config.h"
#include "access_reliable.h"
#include "device_state_manager.h"
#include "nrf_mesh.h"
#include "nrf_mesh_assert.h"
#include "log.h"


/*****************************************************************************
 * Static variables
 *****************************************************************************/

/** Keeps a single global TID for all transfers. */
static uint8_t m_tid;

/*****************************************************************************
 * Static functions
 *****************************************************************************/

/** Returns @c true if the message received was from the address corresponding to the clients
 * publish address. */
 /*
static bool is_valid_source(const simple_battery_client_t * p_client,
                            const access_message_rx_t * p_message)
{
    // Check the originator of the status. 
    dsm_handle_t publish_handle;
    nrf_mesh_address_t publish_address;
    if (access_model_publish_address_get(p_client->model_handle, &publish_handle) != NRF_SUCCESS ||
        publish_handle == DSM_HANDLE_INVALID ||
        dsm_address_get(publish_handle, &publish_address) != NRF_SUCCESS ||
        publish_address.value != p_message->meta_data.src.value)
    {
        return false;
    }
    else
    {
        return true;
    }
}
*/

/*****************************************************************************
 * Opcode handler callback(s)
 *****************************************************************************/

static void handle_status_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
/*
    simple_battery_client_t * p_client = p_args;
    NRF_MESH_ASSERT(p_client->status_cb != NULL);

    if (!is_valid_source(p_client, p_message))
    {
        return;
    }
    
    simple_battery_msg_status_t * p_status =
        (simple_battery_msg_status_t *) p_message->p_data;

    //p_client->status_data.present_battery = *p_message->p_data;
    
    
    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "msg rx value %u Address %u ??? %u\np_client value %u Address %u", p_message->p_data, *p_message->p_data, &p_message->p_data,
    //                                                                                                p_client->status_data.present_battery, &p_client->status_data.present_battery);
    p_client->status_cb(p_client, SIMPLE_BATTERY_STATUS_RECEIVE, p_message->meta_data.src.value);
    */
}

/*
static void handle_set_unreliable_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    uint32_t err_code;

    //simple_battery_rcv_set_unreliable_t rcv_set_unreliable;
    simple_battery_client_t * p_client = p_args;

    
    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Receive ADC Address %u , Tid Address %u, Device ID Address %u\n", p_message->p_data, p_message->p_data+1, p_message->p_data+2);
   // __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Receive ADC Value %d , Tid %d, Device ID %04x\n", rcv_set_unreliable.battery, rcv_set_unreliable.tid, rcv_set_unreliable.device_ID);
    
    p_client->set_unreliable_cb(p_client, *(p_message->p_data), p_message->meta_data.src.value);
}
*/
static const access_opcode_handler_t m_opcode_handlers[] =
{
    {ACCESS_OPCODE_VENDOR(SIMPLE_BATTERY_OPCODE_STATUS, SIMPLE_BATTERY_COMPANY_ID), handle_status_cb},
    //{ACCESS_OPCODE_VENDOR(SIMPLE_BATTERY_OPCODE_SET_UNRELIABLE, SIMPLE_BATTERY_COMPANY_ID), handle_set_unreliable_cb}
};

static void handle_publish_timeout(access_model_handle_t handle, void * p_args)
{
    simple_battery_client_t * p_client = p_args;

    if (p_client->timeout_cb != NULL)
    {
        p_client->timeout_cb(handle, p_args);
    }
}

/*****************************************************************************
 * Public API
 *****************************************************************************/

uint32_t simple_battery_client_init(simple_battery_client_t * p_client, uint16_t element_index)
{
    if (p_client == NULL)
    {
        return NRF_ERROR_NULL;
    }

    access_model_add_params_t init_params;
    init_params.model_id.model_id = SIMPLE_BATTERY_CLIENT_MODEL_ID;
    init_params.model_id.company_id = SIMPLE_BATTERY_COMPANY_ID;
    init_params.element_index = element_index;
    init_params.p_opcode_handlers = &m_opcode_handlers[0];
    init_params.opcode_count = sizeof(m_opcode_handlers) / sizeof(m_opcode_handlers[0]);
    init_params.p_args = p_client;
    init_params.publish_timeout_cb = handle_publish_timeout;
    return access_model_add(&init_params, &p_client->model_handle);
}

uint32_t simple_battery_client_set_unreliable(simple_battery_client_t * p_client, uint8_t value, uint8_t repeats, uint16_t src)
{
    simple_battery_msg_set_unreliable_t set_unreliable;
    set_unreliable.battery = value;
    set_unreliable.tid = m_tid++;
    set_unreliable.msg_src = src;
    

    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Value %d, SRC 0x%02x%02x\n", set_unreliable.battery, set_unreliable.src_H, set_unreliable.src_L);
    
    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Value %d, sb %d, &sb %u, &st %u, *&s %d, *&st %d\n", value, set_unreliable.battery, &set_unreliable.battery, &set_unreliable.tid, *&set_unreliable, *&set_unreliable.tid);

    access_message_tx_t message;
    message.opcode.opcode = SIMPLE_BATTERY_OPCODE_SET_UNRELIABLE;
    message.opcode.company_id = SIMPLE_BATTERY_COMPANY_ID;
    message.p_buffer = (const uint8_t*) &set_unreliable;
    message.length = sizeof(set_unreliable);
    message.force_segmented = false;
    message.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;

    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Set Group Address Data mp %d, &mp %d, *&mp %d\n", message.p_buffer, &message.p_buffer, *&message.p_buffer);

    uint32_t status = NRF_SUCCESS;
    for (uint8_t i = 0; i < repeats; ++i)
    {
        message.access_token = nrf_mesh_unique_token_get();
        status = access_model_publish(p_client->model_handle, &message);
        if (status != NRF_SUCCESS)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Error\n");
            break;
        }
    }
    return status;
}
