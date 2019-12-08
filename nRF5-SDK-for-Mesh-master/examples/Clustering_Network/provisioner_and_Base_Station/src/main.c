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

#include <stdint.h>
#include <string.h>

/* HAL */
#include "boards.h"
#include "nrf_delay.h"
#include "simple_hal.h"
#include "app_timer.h"
#include "nrf_uart.h"
#include "app_uart.h"
#include "nrf_drv_saadc.h"

/* Core */
#include "nrf_mesh.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh_assert.h"
#include "access_config.h"
#include "device_state_manager.h"
#include "flash_manager.h"
#include "mesh_stack.h"
#include "net_state.h"

/* Provisioning and configuration */
#include "provisioner_helper.h"
#include "node_setup.h"
#include "mesh_app_utils.h"
#include "mesh_softdevice_init.h"

/* Models */
#include "config_client.h"
#include "config_server.h"
#include "health_client.h"
//#include "simple_on_off_client.h"
#include "simple_battery_server.h"
#include "emergency_help_server.h"


/* Logging and RTT */
#include "rtt_input.h"
#include "log.h"

/* Example specific includes */
#include "light_switch_example_common.h"
#include "example_network_config.h"
#include "nrf_mesh_config_examples.h"

#include "advertiser.h"

#define LORA_ENABLE

#define RTT_INPUT_POLL_PERIOD_MS (100)
#define LED_BLINK_INTERVAL_MS    (200)
#define LED_BLINK_CNT_START      (2)
#define LED_BLINK_CNT_PROV       (4)

#define APP_NETWORK_STATE_ENTRY_HANDLE (0x0001)
#define APP_FLASH_PAGE_COUNT           (1)

#define APP_PROVISIONING_LED            BSP_LED_0
#define APP_CONFIGURATION_LED           BSP_LED_1


//Battery timer publish interval.
#define BAT_PUBLISH_INTERVAL      APP_TIMER_TICKS(60000)                          /**< 1(s)* 60* 30*/
//ADV Broadcast interval.
#define ADV_FIRST_BROADCAST_INTERVAL    APP_TIMER_TICKS(5000)                         /**< 1(s)*/
#define ADV_NORMAL_BROADCAST_INTERVAL   APP_TIMER_TICKS(600000)                          /**< 1(min)*/

//Lora UART buffer
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

#define LORA_ARRAY_COUNT  5


#define ADVERTISER_BUF_SIZE (64)
#define BEACON_INTERVAL     (500)


APP_TIMER_DEF(m_bat_publish_id);
APP_TIMER_DEF(m_first_adv_broadcast_id);        //First start.
APP_TIMER_DEF(m_normal_adv_broadcast_id);       //Normal.

/* Required for the provisioner helper module */
static network_dsm_handles_data_volatile_t m_dev_handles;
static network_stats_data_stored_t m_nw_state;

static const uint8_t m_client_node_uuid[NRF_MESH_UUID_SIZE] = CLIENT_NODE_UUID;
static const uint8_t m_server_uuid_filter[SERVER_NODE_UUID_PREFIX_SIZE] = SERVER_NODE_UUID_PREFIX;
static prov_helper_uuid_filter_t m_exp_uuid;
static bool m_node_prov_setup_started;
static advertiser_t m_advertiser;
static uint8_t      m_adv_buffer[ADVERTISER_BUF_SIZE];



/* Forward declarations */
static void app_health_event_cb(const health_client_t * p_client, const health_client_evt_t * p_event);
static void app_config_successful_cb(void);
static void app_config_failed_cb(void);
static void app_mesh_core_event_cb (const nrf_mesh_evt_t * p_evt);

static void app_start(void);

//static bool lora_trans_normal(uint8_t *data, uint16_t length, uint8_t priority);
static void lora_trans_array(uint8_t *data, uint16_t length, uint8_t priority);
static void lora_trans(uint8_t priority, uint16_t length);

static void adv_init(void);
static void ch_first_adv_start(void);
static void timer_start(void);


static nrf_mesh_evt_handler_t m_mesh_core_event_handler = { .evt_cb = app_mesh_core_event_cb };

static simple_battery_server_t m_bat_server[1];
static emergency_help_server_t m_emh_server[1];

static nrf_saadc_value_t adc_value;

static uint8_t lora_trans_flag = 0;       //0:standby; 1:bat; 2:emh.

static bool first_flag = 1;
static uint8_t tid = 1;

/**
 * Store 3 LoRa data(s).
 * Array[n][0] : LoRa trans flag.0 have set. 1 not set.
 * Array[n][1-19] : Lora trans data.
 */
static uint8_t lora_Array[LORA_ARRAY_COUNT][20] = {0};
static uint8_t lora_trans_array_num = 0;

static uint8_t EMH_count[10];
static bool first_start = 0;
static uint8_t count = 0;
static uint8_t array_num = 0;

/*****************************************************************************/
/**** Flash handling ****/
#if PERSISTENT_STORAGE

static flash_manager_t m_flash_manager;

static void app_flash_manager_add(void);

static void flash_write_complete(const flash_manager_t * p_manager, const fm_entry_t * p_entry, fm_result_t result)
{
     __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Flash write complete\n");

    /* If we get an AREA_FULL then our calculations for flash space required are buggy. */
    NRF_MESH_ASSERT(result != FM_RESULT_ERROR_AREA_FULL);

    /* We do not invalidate in this module, so a NOT_FOUND should not be received. */
    NRF_MESH_ASSERT(result != FM_RESULT_ERROR_NOT_FOUND);
    if (result == FM_RESULT_ERROR_FLASH_MALFUNCTION)
    {
        ERROR_CHECK(NRF_ERROR_NO_MEM);
    }
}

static void flash_invalidate_complete(const flash_manager_t * p_manager, fm_handle_t handle, fm_result_t result)
{
    /* This application does not expect invalidate complete calls. */
    ERROR_CHECK(NRF_ERROR_INTERNAL);
}

typedef void (*flash_op_func_t) (void);
static void flash_manager_mem_available(void * p_args)
{
    ((flash_op_func_t) p_args)(); /*lint !e611 Suspicious cast */
}


static void flash_remove_complete(const flash_manager_t * p_manager)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Flash remove complete\n");
}

static void app_flash_manager_add(void)
{

    static fm_mem_listener_t flash_add_mem_available_struct = {
        .callback = flash_manager_mem_available,
        .p_args = app_flash_manager_add
    };
    flash_manager_config_t manager_config;
    manager_config.write_complete_cb = flash_write_complete;
    manager_config.invalidate_complete_cb = flash_invalidate_complete;
    manager_config.remove_complete_cb = flash_remove_complete;
    manager_config.min_available_space = WORD_SIZE;
    manager_config.p_area = (const flash_manager_page_t *) (((const uint8_t *) dsm_flash_area_get()) - (ACCESS_FLASH_PAGE_COUNT * PAGE_SIZE * 2));
    manager_config.page_count = APP_FLASH_PAGE_COUNT;
    uint32_t status = flash_manager_add(&m_flash_manager, &manager_config);
    if (NRF_SUCCESS != status)
    {
        flash_manager_mem_listener_register(&flash_add_mem_available_struct);
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Unable to add flash manager for app data\n");
    }
}

static bool load_app_data(void)
{
    flash_manager_wait();
    const fm_entry_t * p_entry = flash_manager_entry_get(&m_flash_manager, APP_NETWORK_STATE_ENTRY_HANDLE);
    if (p_entry == NULL)
    {
        memset(&m_nw_state, 0x00, sizeof(m_nw_state));
        return false;
    }

    memcpy(&m_nw_state, p_entry->data, sizeof(m_nw_state));
    return true;
}

static uint32_t store_app_data(void)
{
    fm_entry_t * p_entry = flash_manager_entry_alloc(&m_flash_manager, APP_NETWORK_STATE_ENTRY_HANDLE, sizeof(m_nw_state));
    static fm_mem_listener_t flash_add_mem_available_struct = {
        .callback = flash_manager_mem_available,
        .p_args = store_app_data
    };

    if (p_entry == NULL)
    {
        flash_manager_mem_listener_register(&flash_add_mem_available_struct);
    }
    else
    {
        network_stats_data_stored_t * p_nw_state = (network_stats_data_stored_t *) p_entry->data;
        memcpy(p_nw_state, &m_nw_state, sizeof(m_nw_state));
        flash_manager_entry_commit(p_entry);
    }

    return NRF_SUCCESS;
}

static void clear_app_data(void)
{
    memset(&m_nw_state, 0x00, sizeof(m_nw_state));

    if (flash_manager_remove(&m_flash_manager) != NRF_SUCCESS)
    {
        /* Register the listener and wait for some memory to be freed up before we retry. */
        static fm_mem_listener_t mem_listener = {.callback = flash_manager_mem_available,
                                                 .p_args = clear_app_data};
        flash_manager_mem_listener_register(&mem_listener);
    }
}

#else

static void clear_app_data(void)
{
    return;
}

bool load_app_data(void)
{
    return false;
}
static uint32_t store_app_data(void)
{
    return NRF_SUCCESS;
}

#endif


static void app_data_store_cb(void)
{
    ERROR_CHECK(store_app_data());
}

/*****************************************************************************/
/**** Configuration process related callbacks ****/

static void app_config_successful_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Configuration of device %u successful\n", m_nw_state.configured_devices);

    hal_led_pin_set(APP_CONFIGURATION_LED, 0);

    m_nw_state.configured_devices++;
    access_flash_config_store();
    ERROR_CHECK(store_app_data());

    if (m_nw_state.configured_devices < (SERVER_NODE_COUNT + CLIENT_NODE_COUNT))
    {
        m_exp_uuid.p_uuid = m_server_uuid_filter;
        m_exp_uuid.length = SERVER_NODE_UUID_PREFIX_SIZE;
        prov_helper_provision_next_device(PROVISIONER_RETRY_COUNT, m_nw_state.next_device_address, &m_exp_uuid);
        prov_helper_scan_start();

        hal_led_pin_set(APP_PROVISIONING_LED, 1);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "All servers provisioned\n");

        hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
    }
}

static void app_config_failed_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Configuration of device %u failed. Press Button 1 to retry.\n", m_nw_state.configured_devices);
    m_node_prov_setup_started = false;
    hal_led_pin_set(APP_CONFIGURATION_LED, 0);
}

static void app_prov_success_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Provisioning successful\n");

    hal_led_pin_set(APP_PROVISIONING_LED, 0);
    hal_led_pin_set(APP_CONFIGURATION_LED, 1);
}

static void app_prov_failed_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Provisioning failed. Press Button 1 to retry.\n");
    m_node_prov_setup_started = false;

    hal_led_pin_set(APP_PROVISIONING_LED, 0);
}


/*****************************************************************************/
/**** Model related callbacks ****/
static void app_health_event_cb(const health_client_t * p_client, const health_client_evt_t * p_event)
{
    switch (p_event->type)
    {
        case HEALTH_CLIENT_EVT_TYPE_CURRENT_STATUS_RECEIVED:
            /*
            __LOG(LOG_SRC_APP,
                  LOG_LEVEL_INFO,
                  "Node 0x%04x alive with %u active fault(s), RSSI: %d\n",
                  p_event->p_meta_data->src.value,
                  p_event->data.fault_status.fault_array_length,
                  ((p_event->p_meta_data->p_core_metadata->source == NRF_MESH_RX_SOURCE_SCANNER)
                       ? p_event->p_meta_data->p_core_metadata->params.scanner.rssi
                       : 0));
                       */
            break;
        default:
            break;
    }
}

static void app_config_server_event_cb(const config_server_evt_t * p_evt)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "config_server Event %d.\n", p_evt->type);

    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        /* This should never return */
        hal_device_reset(0);
    }
}

static void app_config_client_event_cb(config_client_event_type_t event_type, const config_client_event_t * p_event, uint16_t length)
{
    /* USER_NOTE: Do additional processing of config client events here if required */
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Config client event\n");

    /* Pass events to the node setup helper module for further processing */
    node_setup_config_client_event_process(event_type, p_event, length);
}


/** Check if all devices have been provisioned. If not, provision remaining devices.
 *  Check if all devices have been configured. If not, start configuring them.
 */
static void check_network_state(void)
{
    if (!m_node_prov_setup_started)
    {
        /* If previously provisioned device is not configured, start node setup procedure. */
        if (m_nw_state.configured_devices < m_nw_state.provisioned_devices)
        {
            /* Execute configuration */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Waiting for provisioned node to be configured ...\n");
            node_setup_start(m_nw_state.last_device_address, PROVISIONER_RETRY_COUNT,
                            m_nw_state.appkey, APPKEY_INDEX);

            hal_led_pin_set(APP_CONFIGURATION_LED, 1);
        }
        /*
        else if (m_nw_state.provisioned_devices == 0)
        {
            /* Start provisioning - First provision the client with known UUID 
            m_exp_uuid.p_uuid = m_client_node_uuid;
            m_exp_uuid.length = NRF_MESH_UUID_SIZE;
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Waiting for Client node to be provisioned ...\n");
            prov_helper_provision_next_device(PROVISIONER_RETRY_COUNT, m_nw_state.next_device_address, &m_exp_uuid);
            prov_helper_scan_start();

            hal_led_pin_set(APP_PROVISIONING_LED, 1);
        }
        */
        else if (m_nw_state.provisioned_devices < (SERVER_NODE_COUNT + CLIENT_NODE_COUNT))
        {
            /* Start provisioning - rest of the devices */
            m_exp_uuid.p_uuid = m_server_uuid_filter;
            m_exp_uuid.length = SERVER_NODE_UUID_PREFIX_SIZE;
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Waiting for Server node to be provisioned ...\n");
            prov_helper_provision_next_device(PROVISIONER_RETRY_COUNT, m_nw_state.next_device_address, &m_exp_uuid);
            prov_helper_scan_start();

            hal_led_pin_set(APP_PROVISIONING_LED, 1);
        }
        else
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "All servers provisioned\n");
            return;
        }

        m_node_prov_setup_started = true;
    }
    else
    {
         __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Waiting for previous procedure to finish ...\n");
    }
}

static void app_mesh_core_event_cb (const nrf_mesh_evt_t * p_evt)
{
    /* USER_NOTE: User can insert mesh core event proceesing here */
    switch(p_evt->type)
    {
        /* Start user application specific functions only when flash is stable */
        case NRF_MESH_EVT_FLASH_STABLE:
            __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "Mesh evt: FLASH_STABLE \n");
#if (PERSISTENT_STORAGE)
            {
                static bool s_app_started;
                if (!s_app_started)
                {
                    /* Flash operation initiated during initialization has been completed */
                    app_start();
                    s_app_started = true;
                }
            }
#endif
            break;

        default:
            __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "Unhandled Mesh Event: %d \n", p_evt->type);
            break;
    }
}

/* Binds the local models correctly with the desired keys */
void app_default_models_bind_setup(void)
{
    //static dsm_handle_t        m_group_addr_handle = DSM_HANDLE_INVALID;

    /* Bind health client to App key, and configure publication key */
    ERROR_CHECK(access_model_application_bind(m_dev_handles.m_health_client_instance.model_handle, m_dev_handles.m_appkey_handle));
    ERROR_CHECK(access_model_publish_application_set(m_dev_handles.m_health_client_instance.model_handle, m_dev_handles.m_appkey_handle));

    /* Bind self-config server to the self device key */
    ERROR_CHECK(config_server_bind(m_dev_handles.m_self_devkey_handle));
}

/* Binds the battery and emh models correctly with the desired keys */
void app_customer_models_bind_setup(void)
{
    static dsm_handle_t        m_group_addr_handle = DSM_HANDLE_INVALID;

    /* Bind battery server to App key, and configure publication key */
    ERROR_CHECK(dsm_address_subscription_add(GROUP_ADDRESS_TEST, &m_group_addr_handle));
    
    ERROR_CHECK(access_model_application_bind(m_bat_server[0].model_handle, m_dev_handles.m_appkey_handle));
    ERROR_CHECK(access_model_subscription_add(m_bat_server[0].model_handle, m_group_addr_handle));

    ERROR_CHECK(access_model_application_bind(m_emh_server[0].model_handle, m_dev_handles.m_appkey_handle));
}

static bool app_flash_config_load(void)
{
    bool app_load = false;
#if PERSISTENT_STORAGE
    app_flash_manager_add();
    app_load = load_app_data();
#endif
    if (!app_load)
    {
        m_nw_state.provisioned_devices = 0;
        m_nw_state.configured_devices = 0;
        m_nw_state.next_device_address = UNPROV_START_ADDRESS;
        ERROR_CHECK(store_app_data());
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Restored: App data\n");
    }
    return app_load;
}

static void button_event_handler(uint32_t button_number)
{
    static bool flag = 0;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number + 1);
    switch (button_number)
    {
        case 0:
        {
            /* Check if all devices have been provisioned or not */
            check_network_state();
            break;
        }

        case 1:
        {
            prov_helper_scan_stop();

            break;
        }

        case 2:
        {
        /*
            if(flag)
            {
                app_timer_stop(m_bat_publish_id);
            }else timer_start();

            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "BTN Flag %d\n", flag);

            flag ^= 1;
        */
            timer_start();
            ch_first_adv_start();
            
            break;
        }

        /* Initiate node reset */
        case 3:
        {
            /* Clear all the states to reset the node. */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset -----\n");

            clear_app_data();
            mesh_stack_config_clear();

            hal_led_blink_ms(1 << BSP_LED_3, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Press reset button or power cycle the device  -----\n");
            break;
        }

        default:
            break;
    }
}

static void app_rtt_input_handler(int key)
{
    if (key >= '0' && key <= '4')
    {
        uint32_t button_number = key - '0';
        button_event_handler(button_number);
    }
}

/******************************************************************************/
/*                      Battery Model Callback                                */
/******************************************************************************/
static void bat_server_set_group_cb(const simple_battery_server_t *p_self, uint8_t bat_value, uint16_t src)
{
    uint8_t lora_data[19];

    snprintf(lora_data, sizeof(lora_data), "AT+DTX=8,%04x%04d\n", src, bat_value);

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "%s\n", lora_data);

    #ifdef LORA_ENABLE
    lora_trans_array(lora_data, sizeof(lora_data), 1);
    #endif
}


/******************************************************************************/
/*                          Emergency Help Model Callback                     */
/******************************************************************************/

static bool emh_server_set_cb(const emergency_help_server_t * p_self, uint16_t en_emh, uint16_t src)
{
    uint8_t lora_data[19];

    snprintf(lora_data, sizeof(lora_data),"AT+DTX=8,%04x%04d\n", src, en_emh);
    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "lora_data : %s, length : %d\n", lora_data, sizeof(lora_data));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "%s\n", lora_data);
    //if(lora_trans_flag < 2) 
    //lora_trans_normal(lora_data, sizeof(lora_data), 2);
    
    //lora_trans_array(lora_data, sizeof(lora_data), 2);

    return 1;
}

static void emh_server_unreliable_cb(const emergency_help_server_t *p_self, uint16_t src)
{
    uint8_t lora_data[19];
    
    count ++;

    snprintf(lora_data, sizeof(lora_data),"AT+DTX=8,%04x0707\n", src);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "EMH UN_MSG SRC : 0x%04x\n",  src);
    
    #ifdef LORA_ENABLE
   lora_trans_array(lora_data, sizeof(lora_data), 2);
    #endif
    
}

/******************************************************************************/
/*                          Model Initial Callback                     */
/******************************************************************************/

void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");
    m_dev_handles.m_netkey_handle = DSM_HANDLE_INVALID;
    m_dev_handles.m_appkey_handle = DSM_HANDLE_INVALID;
    //m_dev_handles.m_group_handle  = DSM_HANDLE_INVALID;
    m_dev_handles.m_self_devkey_handle = DSM_HANDLE_INVALID;

    /* This app requires following models :
     * config client : To be able to configure other devices
     * health client : To be able to interact with other health servers */
    ERROR_CHECK(config_client_init(app_config_client_event_cb));
    ERROR_CHECK(health_client_init(&m_dev_handles.m_health_client_instance, 0, app_health_event_cb));

    //add battery server model 
    m_bat_server[0].set_unreliable_cb = bat_server_set_group_cb;
    ERROR_CHECK(simple_battery_server_init(&m_bat_server[0], 1));
    ERROR_CHECK(access_model_subscription_list_alloc(m_bat_server[0].model_handle));

    m_emh_server[0].set_cb = emh_server_set_cb;
    m_emh_server[0].set_unreliable_cb = emh_server_unreliable_cb;
    ERROR_CHECK(emergency_help_server_init(&m_emh_server[0], 1));
    ERROR_CHECK(access_model_subscription_list_alloc(m_emh_server[0].model_handle));
    
}

static void mesh_init(void)
{
    bool device_provisioned;
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = app_config_server_event_cb
    };
    ERROR_CHECK(mesh_stack_init(&init_params, &device_provisioned));

    nrf_mesh_evt_handler_add(&m_mesh_core_event_handler);

    /* Load application configuration, if available */
    m_dev_handles.flash_load_success = app_flash_config_load();

    /* Initialize the provisioner */
    mesh_provisioner_init_params_t m_prov_helper_init_info =
    {
        .p_dev_data = &m_dev_handles,
        .p_nw_data = &m_nw_state,
        .netkey_idx = NETKEY_INDEX,
        .p_data_store_cb  = app_data_store_cb,
        .p_prov_success_cb = app_prov_success_cb,
        .p_prov_failed_cb = app_prov_failed_cb
    };
    prov_helper_init(&m_prov_helper_init_info);

    if (!device_provisioned)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Setup defaults: Adding keys, addresses, and bindings \n");

        prov_helper_provision_self();
        app_default_models_bind_setup();
        app_customer_models_bind_setup();
        access_flash_config_store();
        app_data_store_cb();
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Restored: Handles \n");
        prov_helper_device_handles_load();
    }

    node_setup_cb_set(app_config_successful_cb, app_config_failed_cb);

    adv_init();
}


//**************                              APP Init                  *************************/

static bool lora_trans_normal(uint8_t *data, uint16_t length, uint8_t priority)
{
    uint32_t err_code;

    if(priority > lora_trans_flag)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "lora trans Data : %s, Len : %d\n", data, length);

        app_uart_put(0x0D);
    
        for(uint8_t i =0; i<length; i++)
        {
          do
            {
              err_code = app_uart_put(data[i]);
              if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,"Failed receiving NUS message. Error 0x%x. ", err_code);
                    APP_ERROR_CHECK(err_code);
                }else
                {
                  __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,"%c\n", data[i]);
                    //if(lora_data[i] == 0x0A) __LOG(NULL, NULL,"\n");
                }
            }while (err_code == NRF_ERROR_BUSY);
      }
            lora_trans_flag = priority;
    }

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "LoRa trans, flag %d\n", lora_trans_flag);
   return 1;
}



/**
 * !!!BUG!!! : uart num = array num,uart error.
 */
static void lora_trans_array(uint8_t *data, uint16_t length, uint8_t priority)
{
   static uint8_t i = 0;

    if(priority == 2)
    {
        EMH_count[i] = array_num;
        i++;
        i = i%10;
    }
    
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "lora_Array[%d], Len : %d, lora trans Data : %s\n", array_num, length, data);
    
    
    lora_Array[array_num][0] = length;

    for(uint8_t i=1; i<20; i++)
    {
        lora_Array[array_num][i] = *(data+i-1);
        //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO,"I : %d, Array : %c\n", i, lora_Array[array_num][i]);
    }

    lora_trans(1,length); 
    array_num++;
    array_num = array_num % LORA_ARRAY_COUNT;
    
}

static void lora_trans(uint8_t priority, uint16_t length)
{
    uint32_t err_code;

    if(priority > lora_trans_flag)
    {
        app_uart_put(0x0D);
         
    
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,"NRF_TO_6509_DONE\n");
        lora_trans_flag = priority;
        for(uint8_t i =0; i<length; i++)
        {
          do
          {   
            err_code = app_uart_put(lora_Array[lora_trans_array_num][i+1]);
            if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
              {
                  __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,"Failed receiving NUS message. Error 0x%x. ", err_code);
                  APP_ERROR_CHECK(err_code);
              }else
              {
                //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO,"%c\n", lora_Array[lora_trans_array_num][i+1]);
              }
          }while (err_code == NRF_ERROR_BUSY);
        }
       
        lora_Array[lora_trans_array_num][0] = 0;

    }else
    {
       //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Priority %d < Lora_trans_flag%d, LoRa trans flag %d\n", priority, lora_trans_flag, lora_trans_flag); 
      __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,"BUSY\n")
    }
}

//**-----------------------------------------Driver Handler-------------------------------------------------------**/
void uart_event_handle(app_uart_evt_t * p_event)
{
    uint8_t        data[100];
    static uint8_t        index = 0;
    //static uint8_t count = 0;
    static uint8_t i = 0;
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }else if (p_event->evt_type == APP_UART_DATA_READY)
    {
        UNUSED_VARIABLE(app_uart_get(&data[index]));

        //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "%d - %c\n", index, data[index]);
        if(data[index-4] == 'E' && data[index-3] == 'R' && data[index-2] == 'R' && data[index-1] == 'O' && data[index] == 'R')
        {
          lora_trans_flag = 0;
        }
  /*----------------------------------Detect SNR---------------------------------------------------------*/
        if(data[index-2] == 'S' && data[index-1] == 'N' && data[index] == 'R')
        {
            
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,"Get_SNR\n");
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,"count:%d\n",count);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,"Get_Lora_array_num:%d , array_num:%d emh_count:%d \n",lora_trans_array_num, array_num,EMH_count[i]);
            lora_trans_flag = 0;
            if(count > 0)
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,"NEXT_Trans_EMH\n");
                lora_trans_array_num = EMH_count[i];
                lora_trans_array_num = lora_trans_array_num % LORA_ARRAY_COUNT;
                count--;
                i++;
                i = i % 10;
            }else{
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,"NEXT_Trans_BAT\n");
                lora_trans_array_num++;
                lora_trans_array_num = lora_trans_array_num % LORA_ARRAY_COUNT;
            }
            //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Rx OK, flag : %d, Lora_array_num: %d, First_start: %d \n", lora_trans_flag, lora_trans_array_num,first_start);
        }
/*----------------------------------Detect RadioTxDelayDone---------------------------------------------------------*/
        if(data[index-2] == 'D' && data[index-1] == 'e' && data[index] == 'l')
        {
            //lora_rx_flag ++ ;
            lora_trans_flag = 0;
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,"Tx_Delay Down\n");
            if( lora_Array[lora_trans_array_num][0] != 0)
            {
              __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "BUFFER HAVENT SEND\n");
              lora_trans_flag = 0;
              lora_trans(2,19);
            }else
            {
              __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "READY RX\n");
            }   
        }
 /*----------------------------------Detect Radio Tx Timeout  ---------------------------------------------------------*/     
        if(data[index-2] == 'o' && data[index-1] == 'u' && data[index] == 't')
        {
          __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "TimeOUT ReBack Trans\n");
          lora_trans_flag = 0;
          lora_trans(2,19);         
        }
        
        
        if(data[index] == '\n')
        {
            index = 0;
        }else
        {
            index++;
        }
    }
}

static void battery_publish_timeout_handler(void *p_context)
{
    static uint8_t status;
    static uint32_t times = 0;

    //device_ID = ascii_to_uint((uint8_t *)DEVICE_NAME);
    //uint8_t value;

    times = times%3; //1:1.2:2.3:3......(min)
    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Times %d\n", times);
    
    if(!times){
        nrf_drv_saadc_sample_convert(0, &adc_value);
        /*
        status = simple_battery_client_set_unreliable(&m_bat_clients[0], adc_value, GROUP_MSG_REPEAT_COUNT, deviceID);
        if (status == NRF_SUCCESS)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Time-out Detect ADC Value %d\n", adc_value);
        }
        */
    }
    times ++;

}
//1times/min. Total 3min.
static void adv_first_broadcast_timeout_handler(void *p_context)
{   
    static uint8_t cnt=0;
    static uint32_t err_code;

    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "ADV_FIRST_ADV_TIMEOUT\n");
    
    if(cnt == 36 && first_flag == 1)
    {
        first_flag = 0;
        app_timer_stop(m_first_adv_broadcast_id);

        err_code = app_timer_start(m_normal_adv_broadcast_id,
                                   ADV_NORMAL_BROADCAST_INTERVAL,
                                   NULL);

        APP_ERROR_CHECK(err_code);

    }else
    {
        if(first_flag) ch_first_adv_start();
        cnt ++;
    }
}

//3time/10min.
static void adv_normal_broadcast_timeout_handler(void *p_context)
{
    ch_first_adv_start();
}
//**************                          Advertising Init              *************************/

static void adv_tx_complete_cb(advertiser_t *p_adv, nrf_mesh_tx_token_t token, uint32_t timestamp)
{
    static uint8_t cnt = 0;

    cnt = cnt%3;

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Tx Complete,Tid %d\n", tid);

    if(p_adv->p_packet != NULL)
    {
        tid++;

        if(first_flag)
        {
            advertiser_packet_discard(p_adv, p_adv->p_packet);
            advertiser_disable(p_adv);            
        }

        if(!first_flag)
        {
            if(cnt ==2)
            {
                //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Disable\n");
                
                advertiser_packet_discard(p_adv, p_adv->p_packet);
                advertiser_disable(p_adv);            
            }
            cnt++;
        }
    }
}


static void adv_init(void)
{
    advertiser_instance_init(&m_advertiser, adv_tx_complete_cb, m_adv_buffer, ADVERTISER_BUF_SIZE);
    advertiser_interval_set(&m_advertiser, BEACON_INTERVAL);
}

//Use first ch election.
static void ch_first_adv_start(void)
{
    advertiser_enable(&m_advertiser);
    static uint8_t adv_data[] = 
    {
        0x02, 0x01, 0x06,
        0x1a, 0xff, 0x00, 0xff,
        0x02, 0x15,
        0x88, 0x1b, 0xa1, 0xb4,
        0x2e, 0x12, 0x67, 0x21,
        0x58, 0xc8, 0x57, 0x6a,
        0xab, 0x7a, 0xa9,
        0xff, 0xff,
        0xff, 0xff,
        0x01,                           //Level.
        0x00                            //Tid
    };

    adv_data[29] = tid;
    
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Tid %d\n", adv_data[29]);

    adv_packet_t *p_packet = advertiser_packet_alloc(&m_advertiser, sizeof(adv_data));
    if(p_packet){
        memcpy(p_packet->packet.payload, adv_data, sizeof(adv_data));

        p_packet->config.repeats = ADVERTISER_REPEAT_INFINITE;
        advertiser_packet_send(&m_advertiser, p_packet);
    }
}

//**************                              HAL Init                  *************************/
static void uart_init()
{
    uint32_t err_code;

    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          NULL,
          NULL,
          APP_UART_FLOW_CONTROL_DISABLED,
          false,
          NRF_UART_BAUDRATE_9600
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_event_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         //APP_IRQ_PRIORITY_MID,
                         err_code);

    APP_ERROR_CHECK(err_code);
}


static void saadc_init()
{
    ret_code_t err_code;
  
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);

    err_code = nrf_drv_saadc_init(NULL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);
}

static void timer_create(void)
{
    uint32_t err_code;

    err_code = app_timer_create(&m_bat_publish_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_publish_timeout_handler);
    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "timer_create err_code : %d\n", err_code);
    APP_ERROR_CHECK(err_code);

    //First Mode.
    err_code = app_timer_create(&m_first_adv_broadcast_id,
                                APP_TIMER_MODE_REPEATED,
                                adv_first_broadcast_timeout_handler);
    //Normal Mode
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_normal_adv_broadcast_id,
                                APP_TIMER_MODE_REPEATED,
                                adv_normal_broadcast_timeout_handler);
}

static void timer_start(void)
{
    uint32_t err_code;

    err_code = app_timer_start(m_first_adv_broadcast_id,
                                ADV_FIRST_BROADCAST_INTERVAL,
                                NULL);

    APP_ERROR_CHECK(err_code);
}

/**
 * Hardware initialize.
 */
static void driver_init(void)
{
    uart_init();
    saadc_init();
    timer_create();
}

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light Switch Provisioner Demo -----\n");

    ERROR_CHECK(app_timer_init());
    hal_leds_init();

#if BUTTON_BOARD
    ERROR_CHECK(hal_buttons_init(button_event_handler));
#endif

    /* Mesh Init */
    nrf_clock_lf_cfg_t lfc_cfg = DEV_BOARD_LF_CLK_CFG;
    ERROR_CHECK(mesh_softdevice_init(lfc_cfg));
    mesh_init();
    driver_init();
}

static void app_start(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Starting application ...\n");
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Provisoned Nodes: %d, Configured Nodes: %d Next Address: 0x%04x\n",
          m_nw_state.provisioned_devices, m_nw_state.configured_devices, m_nw_state.next_device_address);
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Dev key ", m_nw_state.self_devkey, NRF_MESH_KEY_SIZE);
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Net key ", m_nw_state.netkey, NRF_MESH_KEY_SIZE);
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "App key ", m_nw_state.appkey, NRF_MESH_KEY_SIZE);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Press Button 1 to start provisioning and configuration process. \n");
}

static void start(void)
{
    rtt_input_enable(app_rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);
    ERROR_CHECK(nrf_mesh_enable());
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "<start> \n");

#if (!PERSISTENT_STORAGE)
    app_start();
#endif

    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);
}

int main(void)
{
    initialize();
    execution_start(start);

    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "\nBS&Provisioner\nAddress : 0x%04d, Counet %d\n", node_address.address_start, node_address.count);

    for (;;)
    {
        (void)sd_app_evt_wait();
    }
}
