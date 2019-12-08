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


#include "boards.h"
#include "simple_hal.h"
#include "log.h"
#include "access_config.h"
//#include "simple_on_off_client.h"
#include "simple_battery_client.h"
#include "emergency_help_client.h"
#include "rtt_input.h"
#include "device_state_manager.h"
#include "light_switch_example_common.h"
#include "mesh_app_utils.h"
#include "mesh_stack.h"
#include "mesh_softdevice_init.h"
#include "mesh_provisionee.h"
#include "nrf_mesh_config_examples.h"
#include "nrf_mesh_configure.h"
#include "mesh_app_utils.h"

#include "advertiser.h"

#include "mesh_adv.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "ble_conn_params.h"
#include "sdk_config.h"
#include "proxy.h"


#include "nrf_uart.h"

#include "app_timer.h"
#include "app_uart.h"

#include "nrf_drv_saadc.h"

#define RTT_INPUT_POLL_PERIOD_MS (100)
#define GROUP_MSG_REPEAT_COUNT   (1)

#define LED_BLINK_INTERVAL_MS       (200)
#define LED_BLINK_SHORT_INTERVAL_MS (50)
#define LED_BLINK_CNT_START         (2)
#define LED_BLINK_CNT_RESET         (3)
#define LED_BLINK_CNT_PROV          (4)
#define LED_BLINK_CNT_NO_REPLY      (6)

#define DEVICE_NAME                     "Power Node"
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(150,  UNIT_1_25_MS)           /**< Minimum acceptable connection interval. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(250,  UNIT_1_25_MS)           /**< Maximum acceptable connection interval. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(100)                        /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called. */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(2000)                       /**< Time between each call to sd_ble_gap_conn_param_update after the first call. */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define BAT_PUBLISH_INTERVAL     APP_TIMER_TICKS(60000)                          /**< 1(s)* 60* 30*/

#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

#define ADVERTISER_BUF_SIZE (64)
#define BEACON_INTERVAL     (500)

APP_TIMER_DEF(m_bat_publish_id);

static nrf_saadc_value_t adc_value;

static void gap_params_init(void);
static void conn_params_init(void);
static void lora_trans(uint8_t device_ID, uint8_t value);

static void adv_init(void);
static void adv_start(void);
static void adv_discard(void);
static void adv_CH_data(void);

static uint8_t deviceID;

static advertiser_t m_advertiser;

static uint8_t      m_adv_buffer[ADVERTISER_BUF_SIZE];


static void on_sd_evt(uint32_t sd_evt, void * p_context)
{
    (void) nrf_mesh_on_sd_evt(sd_evt);
}

NRF_SDH_SOC_OBSERVER(mesh_observer, NRF_SDH_BLE_STACK_OBSERVER_PRIO, on_sd_evt, NULL);

//static simple_on_off_client_t m_clients[CLIENT_MODEL_INSTANCE_COUNT];
static simple_battery_client_t m_bat_clients[1];
static emergency_help_client_t m_emh_clients[1];
static const uint8_t          m_client_node_uuid[NRF_MESH_UUID_SIZE] = CLIENT_NODE_UUID;
static bool                   m_device_provisioned;
static bool   m_emh_status = 0;  //1: set emh; 0: can set emh.

static uint8_t adv_tx_flag = 0;


static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");

    /* Restores the application parameters after switching from the Provisioning service to the Proxy  */
    gap_params_init();
    conn_params_init();

    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);
    deviceID = node_address.address_start+1;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x Device ID %04x\n", node_address.address_start, deviceID);

    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);

    
}

static uint32_t server_index_get(const emergency_help_client_t * p_client)
{
    uint32_t index = p_client - &m_emh_clients[0];
    NRF_MESH_ASSERT(index < SERVER_NODE_COUNT);
    return index;
}

/******************************************************************************/
/*                          Beacon Broadcast/Scanning Callback                */
/******************************************************************************/
static void rx_cb(const nrf_mesh_adv_packet_rx_data_t *p_rx_data)
{
    char msg[65];

    if (m_device_provisioned)
    {
        if(p_rx_data->adv_type == 0x02 && p_rx_data->p_payload[4] == 0x02 && p_rx_data->p_payload[26] == 0x00)
        //if(0)
        {

            (void)snprintf(msg, sizeof(msg),"RX [@%u]: RSSI: %3d ADV TYPE: %x ADDR: [%02x:%02x:%02x:%02x:%02x:%02x]",
                          p_rx_data->p_metadata->params.scanner.timestamp,
                          p_rx_data->p_metadata->params.scanner.rssi,
                          p_rx_data->adv_type,
                          p_rx_data->p_metadata->params.scanner.adv_addr.addr[0],
                          p_rx_data->p_metadata->params.scanner.adv_addr.addr[1],
                          p_rx_data->p_metadata->params.scanner.adv_addr.addr[2],
                          p_rx_data->p_metadata->params.scanner.adv_addr.addr[3],
                          p_rx_data->p_metadata->params.scanner.adv_addr.addr[4],
                          p_rx_data->p_metadata->params.scanner.adv_addr.addr[5]);
    
            //__LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, msg, p_rx_data->p_payload, p_rx_data->length);

            if(!m_emh_status)
            {
                m_emh_status = true;
                emergency_help_client_set(&m_emh_clients[0]);
            }
        }
    }//else __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Not Provisioned\n");
}

/******************************************************************************/
/*                          Clinet Element Callback                           */
/******************************************************************************/

static void client_publish_timeout_cb(access_model_handle_t handle, void * p_self)
{
     __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Acknowledged send timedout\n");
}

static void client_status_cb(const emergency_help_client_t *p_self, emergency_help_status_t status, uint16_t src)
{
    //uint32_t server_index = server_index_get(p_self);

    switch (status)
    {
        case EMERGENCY_HELP_STATUS_RECEIVE:
            
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, " SRC 0x%04x status RECEIV\n", src);
            
            m_emh_status = false;
            //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "ADC Value %u\n", p_self->status_data.present_battery);
            break;

        case EMERGENCY_HELP_STATUS_ERROR_NO_REPLY:
            
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "No reply from  SRC 0x%04x\n", src);

            m_emh_status = false;
            break;

        case EMERGENCY_HELP_STATUS_CANCELLED:

            __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Message to SRC 0x%04x cancelled\n", src);

            m_emh_status = false;
            break;

        default:

            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Unknown status \n");
            break;
    }
}

static void client_set_unreliable_cb(const simple_battery_client_t *p_self, uint8_t bat_value, uint16_t src)
{
    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "client_set_unreliable_cb\n");

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "%d%d\n", bat_value, src);

    //lora_trans(rcv_set_unreliable.device_ID, rcv_set_unreliable.battery);
}


/*****************************************************************************************/

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);
    /* This function may return if there are ongoing flash operations. */
    mesh_stack_device_reset();
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
}

static void button_event_handler(uint32_t button_number)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);

    uint32_t status = NRF_SUCCESS;
    static uint8_t device_ID;
    
    nrf_drv_saadc_sample_convert(0, &adc_value);

    //device_ID = ascii_to_uint((uint8_t *)DEVICE_NAME);
    
    switch (button_number)
    {
        case 0:
            status = emergency_help_client_set(&m_emh_clients[0]);
                       
            break;

        case 1:
            emergency_help_client_pending_msg_cancel(&m_emh_clients[0]); 
            
            break;
            
        case 2:
            //status = simple_battery_client_get(&m_clients[button_number]);
            /* send a group message to the ODD group, with inverted GPIO pin value */
            
            status = simple_battery_client_set_unreliable(&m_bat_clients[0], adc_value, GROUP_MSG_REPEAT_COUNT, deviceID);
            if (status == NRF_SUCCESS)
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Detect ADC Value %d\n", adc_value);
            }

            
            
            break;
            
        case 3:
            mesh_stack_config_clear();
            node_reset();
            break;

        default:
            break;
    }

    switch (status)
    {
        case NRF_SUCCESS:
            break;

        case NRF_ERROR_NO_MEM:
        case NRF_ERROR_BUSY:
        case NRF_ERROR_INVALID_STATE:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Cannot send - client %u is busy\n", button_number);
            hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
            break;

        case NRF_ERROR_INVALID_PARAM:
            /* Publication not enabled for this client. One (or more) of the following is wrong:
             * - An application key is missing, or there is no application key bound to the model
             * - The client does not have its publication state set
             *
             * It is the provisioner that adds an application key, binds it to the model and sets
             * the model's publication state.
             */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Publication not configured for client %u\n", button_number);
            break;

        default:
            ERROR_CHECK(status);
            break;
    }
}

static void rtt_input_handler(int key)
{
    if (key >= '0' && key <= '3')
    {
        uint32_t button_number = key - '0';
        button_event_handler(button_number);
    }
}

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");
    
    //for (uint32_t i = 0; i < CLIENT_MODEL_INSTANCE_COUNT; ++i)
    //{
        //m_bat_clients[0].status_cb = client_status_cb;
        m_bat_clients[0].set_unreliable_cb = client_set_unreliable_cb;
        m_bat_clients[0].timeout_cb = client_publish_timeout_cb;
        ERROR_CHECK(simple_battery_client_init(&m_bat_clients[0], 1));
        ERROR_CHECK(access_model_subscription_list_alloc(m_bat_clients[0].model_handle));
    //}

    m_emh_clients[0].status_cb = client_status_cb;
    m_emh_clients[0].timeout_cb = client_publish_timeout_cb;
    ERROR_CHECK(emergency_help_client_init(&m_emh_clients[0], 1));
    ERROR_CHECK(access_model_subscription_list_alloc(m_emh_clients[0].model_handle));
    
}

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(p_evt->conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
    else if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_SUCCEEDED)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully updated connection parameters\n");
    }
}

static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

//**-----------------------------------------Driver Handler-------------------------------------------------------**/
void uart_event_handle(app_uart_evt_t * p_event)
{
    uint8_t        data[50];
    uint8_t        index = 0;
    static uint8_t count = 0;

    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }else if (p_event->evt_type == APP_UART_DATA_READY)
    {
        //index++;
        UNUSED_VARIABLE(app_uart_get(&data[index]));
        //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "%c", data[index]);
        if(data[index] == 0x0D)
        {
            count ++;
            if(count == 4)
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Tx Done\n");
                count = 0;
            }
        }
        //index = 0;
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
        
        status = simple_battery_client_set_unreliable(&m_bat_clients[0], adc_value, GROUP_MSG_REPEAT_COUNT, deviceID);
        if (status == NRF_SUCCESS)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Time-out Detect ADC Value %d\n", adc_value);
        }
    }
    times ++;

}

//**-----------------------------------------Stack Initial-------------------------------------------------------**/

static void mesh_init(void)
{
    uint8_t dev_uuid[NRF_MESH_UUID_SIZE];
    uint8_t node_uuid_prefix[SERVER_NODE_UUID_PREFIX_SIZE] = SERVER_NODE_UUID_PREFIX;

    ERROR_CHECK(mesh_app_uuid_gen(dev_uuid, node_uuid_prefix, SERVER_NODE_UUID_PREFIX_SIZE));

    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        //.core.p_uuid             = m_client_node_uuid,
        .core.p_uuid             = dev_uuid,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };
    ERROR_CHECK(mesh_stack_init(&init_params, &m_device_provisioned));
    
    //Start listening for incoming packets.
    nrf_mesh_rx_cb_set(rx_cb);
    
    //Start Advertising beacon
    adv_init();

}

static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);
}

static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    ble_gap_conn_params_t  gap_conn_params;

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

    memset(&cp_init, 0, sizeof(cp_init));
    cp_init.p_conn_params                  = &gap_conn_params;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

//**************                          Advertising Init              *************************/
static void adv_tx_complete_cb(advertiser_t *p_adv, nrf_mesh_tx_token_t token, uint32_t timestamp)
{
    
}

static void adv_init(void)
{
    advertiser_instance_init(&m_advertiser, NULL, m_adv_buffer, ADVERTISER_BUF_SIZE);
    advertiser_interval_set(&m_advertiser, BEACON_INTERVAL);
}

static void adv_start(void)
{
    advertiser_enable(&m_advertiser);
    static const uint8_t adv_data[] = 
    {
        0x02, 0x01, 0x06,
        0x1a, 0xff, 0x00, 0x59,
        0x02, 0x15,
        0x00, 0x11, 0x22, 0x33,
        0x44, 0x55, 0x66, 0x77,
        0x88, 0x99, 0xaa, 0xbb,
        0xcc, 0xdd, 0xee, 0xff,
        0x12, 0x34,
        0x43, 0x21,
        0xff
    };

    adv_packet_t *p_packet = advertiser_packet_alloc(&m_advertiser, sizeof(adv_data));
    if(p_packet){
        memcpy(p_packet->packet.payload, adv_data, sizeof(adv_data));

        p_packet->config.repeats = ADVERTISER_REPEAT_INFINITE;

        advertiser_packet_send(&m_advertiser, p_packet);
    }
}

static void adv_CH_data(void)
{
    //advertiser_packet_discard(&m_advertiser, m_advertiser.p_packet);

    static const uint8_t adv_data[] = 
    {
        0x02, 0x01, 0x06,
        0x1a, 0xff, 0x00, 0x59,
        0x02, 0x15,
        0x00, 0x11, 0x22, 0x33,
        0x44, 0x55, 0x66, 0x77,
        0x88, 0x99, 0xaa, 0xbb,
        0xcc, 0xdd, 0xee, 0xff,
        0x12, 0x34,
        0x43, 0x21,
        0xaa
    };

    adv_packet_t *p_packet = advertiser_packet_alloc(&m_advertiser, sizeof(adv_data));
    if(p_packet){
        memcpy(p_packet->packet.payload, adv_data, sizeof(adv_data));

        p_packet->config.repeats = ADVERTISER_REPEAT_INFINITE;

        advertiser_packet_send(&m_advertiser, p_packet);
    }
}

//**************                              APP Init                  *************************/
static void lora_trans(uint8_t device_ID, uint8_t value)
{
    uint32_t err_code;
    uint8_t lora_data[16];

    sprintf( lora_data, "AT+DTX=6,%03x%03d\n", device_ID, value);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "%s\n", lora_data);
    for(uint8_t i =0; i<16; i++)
    {
        do
            {
                err_code = app_uart_put(lora_data[i]);
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,"Failed receiving NUS message. Error 0x%x. ", err_code);
                    APP_ERROR_CHECK(err_code);
                }/*else
                {
                    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,"%c", lora_data[i]);
                    if(lora_data[i] == 0x0A) __LOG(NULL, NULL,"\n");
                }*/
            } while (err_code == NRF_ERROR_BUSY);
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
  
    nrf_saadc_channel_config_t channel_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);

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

}

static void driver_init()
{
    uart_init();
    saadc_init();
    timer_create();
}

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light Switch Client Demo -----\n");
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Start -----\n");

    ERROR_CHECK(app_timer_init());
    hal_leds_init();

#if BUTTON_BOARD
    ERROR_CHECK(hal_buttons_init(button_event_handler));
#endif
    uint32_t err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    uint32_t ram_start = 0;
    /* Set the default configuration (as defined through sdk_config.h). */
    err_code = nrf_sdh_ble_default_cfg_set(MESH_SOFTDEVICE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    gap_params_init();
    conn_params_init();

    mesh_init();
    driver_init();
}

static void timer_start(void)
{
    uint32_t err_code;

    err_code = app_timer_start(m_bat_publish_id,
                               BAT_PUBLISH_INTERVAL,
                               NULL);
    
    APP_ERROR_CHECK(err_code);
}

static void start(void)
{
    rtt_input_enable(rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);
    ERROR_CHECK(mesh_stack_start());

    dsm_local_unicast_address_t node_address;
    
    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_complete_cb = provisioning_complete_cb,
            .p_device_uri = NULL
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }else{
        dsm_local_unicast_addresses_get(&node_address);
        deviceID = node_address.address_start+1;
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x Device ID %04\n", node_address.address_start, deviceID);
    }

    adv_start();

    const uint8_t *p_uuid = nrf_mesh_configure_device_uuid_get();
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Device UUID ", p_uuid, NRF_MESH_UUID_SIZE);

    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);

    timer_start();
}

int main(void)
{

    initialize();
    execution_start(start);

    for (;;)
    {
        (void)sd_app_evt_wait();
    }
}
