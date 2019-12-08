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
#include "simple_battery_client.h"
#include "simple_battery_server.h"
#include "emergency_help_client.h"
#include "emergency_help_server.h"
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

#define BAT_PUBLISH_INTERVAL            APP_TIMER_TICKS(60000)                          /**< 1(s)* 60* 30*/
#define CH_ELECTION_INTERVAL            APP_TIMER_TICKS(10000)                          /**< 1(s)* 10*/
#define I_AM_CH_INTERVAL                APP_TIMER_TICKS(5000)                           /**< 1(s)* 5*/
#define MN_EXCHANGE_ROUTING_INTERVAL    APP_TIMER_TICKS(5000)                           /**< 1(s)* 5*/

#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

#define ADVERTISER_BUF_SIZE (64)
#define BEACON_INTERVAL     (500)

#define CH_ADV_PREFIX   0x02, 0x56, 0xf6, 0xde, 0xde

#define MAJOR_LOCATE_H  0x00
#define MAJOR_LOCATE_L  0x0C
#define MINOR_LOCATE_H  0x02
#define MINOR_LOCATE_L  0x0C

#define BS_RELAY_TIMEOUT  3     //1 times 10s.
#define CH_ELECTION_TIMEOUT   3
#define I_AM_CH_TIMEOUT       3
#define MN_ROUTING_ADV_TIMEOUT  3



APP_TIMER_DEF(m_bat_publish_id);          //Detect battery value and set unreliable msg to BS.
APP_TIMER_DEF(m_ch_election_id);          //Election CH for advertising.
APP_TIMER_DEF(m_i_am_ch_id);              //I am CH for advertising.
APP_TIMER_DEF(m_mn_ex_routing_id);        //MNs exchange destination for advtising.

static nrf_saadc_value_t adc_value;

static void gap_params_init(void);
static void conn_params_init(void);
static void lora_trans(uint8_t device_ID, uint8_t value);

static void adv_init(void);
static void adv_start(void);
static void adv_discard(void);
static void ch_election_adv(uint8_t max_addr_H, uint8_t max_addr_L, uint8_t max_priority);
static void BS_relay_adv(uint8_t rc_level, uint8_t relay_tid);
static void adv_disable(void);
static void i_am_ch_adv(void);
static void mn_routing_adv(void);

static advertiser_t m_advertiser;

static uint8_t      m_adv_buffer[ADVERTISER_BUF_SIZE];


static void on_sd_evt(uint32_t sd_evt, void * p_context)
{
    (void) nrf_mesh_on_sd_evt(sd_evt);
}

NRF_SDH_SOC_OBSERVER(mesh_observer, NRF_SDH_BLE_STACK_OBSERVER_PRIO, on_sd_evt, NULL);

static simple_battery_client_t m_bat_clients[1];
static simple_battery_server_t m_bat_servers[1];
static emergency_help_client_t m_emh_clients[1];
static emergency_help_server_t m_emh_servers[1];

static bool                   m_device_provisioned;
static bool                   provision_flag = 0;
static bool                   m_emh_status = 0;  //1: set emh; 0: can set emh.

static uint8_t adv_packet_status = 0;               //Advertising status.

//Struct for storing ch election data.
struct cluster_header
{
    uint16_t  max_addr;
    uint8_t   max_priority;
    uint8_t   self_priority;
};

typedef struct cluster_header CH;
static CH ch = {0};

//Struct for MNs exchange data.
/*
struct mns_exchange_path
{
    uint8_t   ch_flag;
    uint16_t  destination;
    uint8_t   hop;
};

typedef struct mns_exchange_path MN_EXCHANGE_PATH;
static MN_EXCHANGE_PATH mn_ex_path = {0, 0, 255};

//Struct for storing ch destination addr data.
struct ch_exchange_path
{
    uint16_t destination;
    uint8_t  level;
};

typedef struct ch_exchange_path CH_EXCHANGE_PATH;
static CH_EXCHANGE_PATH ch_ex_path = {0, 255};
*/
struct __self_node_attribute
{
    uint16_t    destination_addr;         
    uint8_t     level;
    uint16_t    self_addr;
    uint8_t     hop;                    //Use for Exchange routing.
    bool        i_am_ch;                //Self is CH flag. 1 : true;0 : false.
    bool        ch_flag;                //Self can become to CH flag. 1:true;0:false.
};

typedef  struct __self_node_attribute SELF_NODE_ATTRIBUTE;
static SELF_NODE_ATTRIBUTE self_node_attr = {0};

//**-----------------------------------------Application-------------------------------------------------------**/
static uint32_t set_model_publish_address(uint16_t publish_address, access_model_handle_t model_handle)
{
    uint32_t status = NRF_SUCCESS;
    dsm_handle_t publish_address_handle = DSM_HANDLE_INVALID;

    access_model_publish_address_get(model_handle, &publish_address_handle);
    NRF_MESH_ASSERT(dsm_address_publish_remove(publish_address_handle) == NRF_SUCCESS);

    status = dsm_address_publish_add(publish_address, &publish_address_handle); 
    if (status != NRF_SUCCESS) {
        return status;
    } 
    else {
        return access_model_publish_address_set(model_handle, publish_address_handle);
    }
}

static uint16_t get_node_addr(void)
{
    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);

    return node_address.address_start;
}

//Start CH election for advertising process.
static void ch_election_start(void)
{
    uint32_t  err_code;

    err_code = app_timer_start(m_ch_election_id, CH_ELECTION_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

//Start I am CH for advertising process.(CH & MNs routing exchange)
static void i_am_ch_start(void)
{
    uint32_t err_code;

    err_code = app_timer_start(m_i_am_ch_id, I_AM_CH_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

//Start MNs exchange for advertising process.(CH & MNs routing exchange)
static void mn_ex_routing_start(void)
{
    uint32_t err_code;

    err_code = app_timer_start(m_mn_ex_routing_id, MN_EXCHANGE_ROUTING_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


static void bat_publish_start(void)
{
    uint32_t err_code;

    err_code = app_timer_start(m_bat_publish_id,
                               BAT_PUBLISH_INTERVAL,
                               NULL);
    
    APP_ERROR_CHECK(err_code);
}
//**-----------------------------------------------------------------------------------------------------------**/
static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");

    //uint16_t node_address;

    /* Restores the application parameters after switching from the Provisioning service to the Proxy  */
    gap_params_init();
    conn_params_init();

    self_node_attr.self_addr = get_node_addr();

    provision_flag = 1;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x\n", self_node_attr.self_addr);

    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
}

/******************************************************************************/
/*                          Clustering Header Election                        */
/******************************************************************************/
//Need add scan low level feature.
static void ch_election_priority_compare(uint16_t max_addr, uint8_t max_priority, uint16_t other_node)
{
    static uint8_t cnt = 0;
    static uint16_t array[10] = {0};
    static uint8_t adc_priority = 0;
    static uint16_t self_node_addr = 0;
    bool check_flag = 0;
    uint16_t addr_temp = 0;

    //First get self node addr.
    if(!self_node_addr) self_node_addr = get_node_addr();

    for(uint8_t i=0; i<=cnt; i++)
    {
        //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Array addr[%d] 0x%04x :: 0x%04x Other addr\n", i, array[i], other_node);
        if(array[i] == other_node)
        { 
            check_flag = 1;
            
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Check Same Address\n");
        }
    }

    //Self conn priority
    if(!check_flag)
    {
        array[cnt] = other_node;

        cnt++;
        adc_priority = (adc_value - 153)/10;

        ch.self_priority = cnt + adc_priority;

        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Add Conn_Priority, Self Priority %d\n", ch.self_priority);
    }

    //ADC : 256 * 0.6 = 153
    if(adc_value > 180 && self_node_attr.ch_flag == 1)
    {
        //Receive max priority Compare self max priority.
        if(max_priority > ch.max_priority)
        {
            ch.max_priority = max_priority;
            ch.max_addr = max_addr;
        }else if (max_priority == ch.max_priority)
        {
            addr_temp = max_addr;
            //Receive max priority = Self max priority,but receive max addr < Self max addr.
            if(addr_temp < ch.max_addr)
            {
                ch.max_priority = max_priority;
                ch.max_addr = max_addr;
            }
        }

        //self max priority Compare self max priority.
        if(ch.self_priority > ch.max_priority)
        {
            ch.max_priority = ch.self_priority;
            ch.max_addr = other_node;
        }else if (ch.self_priority == ch.max_priority)
        {
            //Receive max priority = Self max priority,but receive max addr < Self max addr.
            if(self_node_addr < ch.max_addr)
            {
                ch.max_priority = ch.self_priority;
                ch.max_addr = self_node_addr;
            }
        }
    }else
    {
        //Receive max priority Compare self max priority.
        if(max_priority > ch.max_priority)
        {
            ch.max_priority = max_priority;
            ch.max_addr = max_addr;
        }else if (max_priority == ch.max_priority)
        {
            addr_temp = max_addr;
            //Receive max priority = Self max priority,but receive max addr < Self max addr.
            if(addr_temp < ch.max_addr)
            {
                ch.max_priority = max_priority;
                ch.max_addr = max_addr;
            }
        }
    }
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Receive MNs ADV\nMax Addr and Priority 0x%04x : %d\nSelf Addr and Priority 0x%04x : %d\n", ch.max_addr, ch.max_priority, self_node_addr, ch.self_priority);
}

/******************************************************************************/
/*                                Exchange Path                               */
/******************************************************************************/
static void mns_exchange_routing(uint8_t ch_flag, uint16_t src, uint8_t hop)
{
    static bool first_flag = 0;
    static uint8_t min_hop = 255;

    if(!first_flag)
    {
        first_flag = 1;
        mn_ex_routing_start();
    }
    
    if(hop < min_hop)
    {
        self_node_attr.hop = min_hop = hop;
        self_node_attr.destination_addr = src;
    }
}

//Input 1:CH 2:MNs
static void ch_exchange_routing(uint8_t input, uint16_t src, uint8_t level)
{
    static uint8_t compare_level = 0;
    
    if(!compare_level) compare_level = self_node_attr.level;

    if(level < compare_level)
    {
        if(input == 1)
        {
          self_node_attr.destination_addr = src;
          compare_level = level;
        }else
        {
          self_node_attr.destination_addr = src;
          compare_level = level;
        }
    }
}


/******************************************************************************/
/*                          Beacon Broadcast/Scanning Callback                */
/******************************************************************************/
static void rx_cb(const nrf_mesh_adv_packet_rx_data_t *p_rx_data)
{
    //char msg[65];

    bool node_check_flag = 0;
    static uint8_t BS_ch_adv_level_compare = 255;
    static uint8_t relay_tid = 0;
    uint32_t addr = 0, max_addr, self_addr;        
/*
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
*/

    if (provision_flag)
    {
        //Receive EMH ADV,and set emh msg.
        if(p_rx_data->p_payload[9] == 0xe2 && p_rx_data->p_payload[10] == 0x1f && p_rx_data->p_payload[11] == 0xc4 && p_rx_data->p_payload[12] == 0x9b && p_rx_data->p_payload[13] == 0xda &&
            p_rx_data->p_payload[29] == 0x00)
        {            
            //Receive EMH Beacon, Set EMH MSG.
            if(!m_emh_status)
            {
                m_emh_status = true;
                emergency_help_client_set_unreliable(&m_emh_clients[0], GROUP_MSG_REPEAT_COUNT, self_node_attr.self_addr);
            }
        }
        
        /**
        * Receive BS ADV and MNs rebroadcast ADV,using to divide level.
        * DIVIDE_LEVEL_PREFIX : 9/15
        * MAJOR_LOCATE  : 24/2
        * MINOR_LOCATE  : 26/2
        * Level         : 27/1
        * Repeat Tid    : 28/1
        */
        if(p_rx_data->p_payload[9] == 0x88 && p_rx_data->p_payload[10] == 0x1b && p_rx_data->p_payload[11] == 0xa1 && p_rx_data->p_payload[12] == 0xb4 &&
           p_rx_data->p_payload[13] == 0x2e && p_rx_data->p_payload[14] == 0x12 && p_rx_data->p_payload[15] == 0x67 && p_rx_data->p_payload[16] == 0x21 &&
           p_rx_data->p_payload[17] == 0x58 && p_rx_data->p_payload[18] == 0xc8 && p_rx_data->p_payload[19] == 0x57 && p_rx_data->p_payload[20] == 0x6a &&
           p_rx_data->p_payload[21] == 0xab && p_rx_data->p_payload[22] == 0x7a && p_rx_data->p_payload[23] == 0xa9 &&
           adv_packet_status != 2 && adv_packet_status != 3 && adv_packet_status != 4)
        {            
            //Same Level
            if( MAJOR_LOCATE_H == p_rx_data->p_payload[24] && MAJOR_LOCATE_L == p_rx_data->p_payload[25] &&
                MINOR_LOCATE_H == p_rx_data->p_payload[26] && MINOR_LOCATE_L == p_rx_data->p_payload[27] &&
                p_rx_data->p_payload[29] > relay_tid)
            {
                relay_tid = p_rx_data->p_payload[29];
                BS_relay_adv(BS_ch_adv_level_compare-1, relay_tid);
            }
            //High Level
            if(p_rx_data->p_payload[28] <= BS_ch_adv_level_compare)
            {
                BS_ch_adv_level_compare = p_rx_data->p_payload[28];
                relay_tid = p_rx_data->p_payload[29];
                self_node_attr.ch_flag = 1;

                BS_relay_adv(BS_ch_adv_level_compare, relay_tid);
            }
        }

        /**
        * CH election compare.
        * CH_ADV_PREFIX : 9/4
        * Max_address   : 14/2
        * Max_priority  : 16/1
        * Self_addr     : 17/2
        *
        * MAJOR_LOCATE  : 25/2
        * MINOR_LOCATE  : 27/2
        */
        if(p_rx_data->p_payload[9] == 0x02 && p_rx_data->p_payload[10] == 0x56 && p_rx_data->p_payload[11] == 0xf6 &&
            p_rx_data->p_payload[12] == 0xde && p_rx_data->p_payload[13] == 0xde &&
            MAJOR_LOCATE_H == p_rx_data->p_payload[25] && MAJOR_LOCATE_L == p_rx_data->p_payload[26] &&
            MINOR_LOCATE_H == p_rx_data->p_payload[27] && MINOR_LOCATE_L == p_rx_data->p_payload[28] &&
            adv_packet_status == 2)
        {
            max_addr  = (p_rx_data->p_payload[14])<<8 | p_rx_data->p_payload[15];
            self_addr = (p_rx_data->p_payload[17])<<8 | p_rx_data->p_payload[18];
             __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Max_Addr 0x%04x :: 0x%04x Slef_Addr\n", max_addr, self_addr);
            ch_election_priority_compare( max_addr, p_rx_data->p_payload[16], self_addr);
        }

        

        //uint16_t node_address;
        //node_address = get_node_addr(); 

        /**
        * Receive CH ADV data,Useing for exchange CH and MNs destination address.
        * I_AM_CH_PREFIX  : 9/12
        * MAJOR_LOCATE    : 21/2
        * MINOR_LOCATE    : 23/2
        * Self_Addr       : 25/2
        * Level           : 27/1
        */
        if(p_rx_data->p_payload[9] == 0xf8 && p_rx_data->p_payload[10] == 0x44 && p_rx_data->p_payload[11] == 0x7e && p_rx_data->p_payload[12] == 0x27 &&
           p_rx_data->p_payload[13] == 0x70 && p_rx_data->p_payload[14] == 0xcc && p_rx_data->p_payload[15] == 0x64 && p_rx_data->p_payload[16] == 0xd8 &&
           p_rx_data->p_payload[17] == 0x35 && p_rx_data->p_payload[18] == 0xbb && p_rx_data->p_payload[19] == 0x0c && p_rx_data->p_payload[20] == 0xe0 )
        {         
            addr = p_rx_data->p_payload[25] << 8 | p_rx_data->p_payload[26];
            //MNs receive CH data.
            if( p_rx_data->p_payload[21] == MAJOR_LOCATE_H && p_rx_data->p_payload[22] == MAJOR_LOCATE_L && 
                p_rx_data->p_payload[23] == MINOR_LOCATE_H && p_rx_data->p_payload[24] == MINOR_LOCATE_L && adv_packet_status != 3)
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "MNs receive CH data.\n");
                mns_exchange_routing(1, addr, 0);
            }
            //CH receive CH data
            if( p_rx_data->p_payload[21] != MAJOR_LOCATE_H || p_rx_data->p_payload[22] != MAJOR_LOCATE_L || 
                p_rx_data->p_payload[23] != MINOR_LOCATE_H || p_rx_data->p_payload[24] != MINOR_LOCATE_L && adv_packet_status == 3)
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "CH receive CH data.\n");
                ch_exchange_routing(1, addr, p_rx_data->p_payload[27]);
            }
        }
         
        /**
        * Receive MNs ADV data,Useing for exchange CH and MNs destination address.
        * I_AM_CH_PREFIX  : 9/12
        * MAJOR_LOCATE    : 21/2
        * MINOR_LOCATE    : 23/2
        * Self_Addr       : 25/2
        * Hop             : 27/1
        * Level           : 28/1
        * Destination addr have to Bs flag : 29/1
        */
        if(p_rx_data->p_payload[9] == 0x38 && p_rx_data->p_payload[10] == 0x32 && p_rx_data->p_payload[11] == 0x89 && p_rx_data->p_payload[12] == 0xbf &&
           p_rx_data->p_payload[13] == 0xde && p_rx_data->p_payload[14] == 0xc2 && p_rx_data->p_payload[15] == 0x71 && p_rx_data->p_payload[16] == 0x84 &&
           p_rx_data->p_payload[17] == 0x4a && p_rx_data->p_payload[18] == 0x47 && p_rx_data->p_payload[19] == 0x24 && p_rx_data->p_payload[20] == 0x82 )
        {
            addr = p_rx_data->p_payload[25] << 8 | p_rx_data->p_payload[26];
            //MNs receive MNs data.
            if( p_rx_data->p_payload[21] == MAJOR_LOCATE_H && p_rx_data->p_payload[22] == MAJOR_LOCATE_L && 
                p_rx_data->p_payload[23] == MINOR_LOCATE_H && p_rx_data->p_payload[24] == MINOR_LOCATE_L && adv_packet_status != 3)
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "MNs receive MNs data.\n");
                mns_exchange_routing(1, addr, p_rx_data->p_payload[27]);
            }
            //CH receive MNs data
            if( p_rx_data->p_payload[21] != MAJOR_LOCATE_H || p_rx_data->p_payload[22] != MAJOR_LOCATE_L || 
                p_rx_data->p_payload[23] != MINOR_LOCATE_H || p_rx_data->p_payload[24] != MINOR_LOCATE_L && adv_packet_status == 3)
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "CH receive MNs data.\n");
                ch_exchange_routing(2, addr, p_rx_data->p_payload[28]);
            }
        }
    }
}

/******************************************************************************/
/*                          Clinet Element Callback                           */
/******************************************************************************/
static void client_publish_timeout_cb(access_model_handle_t handle, void * p_self)
{
     __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Acknowledged send timedout\n");
}

static void emh_client_status_cb(const emergency_help_client_t *p_self, emergency_help_status_t status, uint16_t src)
{
    switch (status)
    {
        case EMERGENCY_HELP_STATUS_RECEIVE:
            
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, " SRC 0x%04x status RECEIV\n", src);
            
            m_emh_status = false;
            
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

/******************************************************************************/
/*                          Server Element Callback                           */
/******************************************************************************/
static void bat_server_set_unreliable_cb(const simple_battery_server_t *p_self, uint8_t bat_value, uint16_t src)
{
    uint32_t status;

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "BAT Value %d,SRC Node 0%04x\n", bat_value, src);
    status = simple_battery_client_set_unreliable(&m_bat_clients[0], adc_value, GROUP_MSG_REPEAT_COUNT, src);
}

static void emh_server_set_unreliable_cb(const emergency_help_server_t *p_self, uint16_t src)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "EMH UN_MSG SRC : 0x%04x\n",  src);
    emergency_help_client_set_unreliable(&m_emh_clients[0], GROUP_MSG_REPEAT_COUNT, src);

}

static bool emh_server_set_cb(const emergency_help_server_t * p_self, uint16_t en_emh, uint16_t src)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "EMH UN_MSG SRC : 0x%04x\n",  src);
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
    static bool test_flag = 1;
    static uint16_t node_addr = 0;

    if(!node_addr) node_addr = get_node_addr();
    
    
    switch (button_number)
    {
        case 0:
            //status = emergency_help_client_set(&m_emh_clients[0]);
            emergency_help_client_set_unreliable(&m_emh_clients[0], GROUP_MSG_REPEAT_COUNT, node_addr);
                       
            break;

        case 1:
            emergency_help_client_pending_msg_cancel(&m_emh_clients[0]); 
            
            break;
            
        case 2:
        
            if(!test_flag)
            {
                test_flag = 1;
                set_model_publish_address(0x0102, m_bat_clients[0].model_handle);
            }
         
            nrf_drv_saadc_sample_convert(0, &adc_value);

            status = simple_battery_client_set_unreliable(&m_bat_clients[0], adc_value, GROUP_MSG_REPEAT_COUNT, node_addr);
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


//Model Elememt Count : nrf_mesh_config_app.h #define ACCESS_ELEMENT_COUNT (count)
static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");
   
    //Initial Battery  Client Model.
    //m_bat_clients[0].set_unreliable_cb = client_set_unreliable_cb;
    m_bat_clients[0].timeout_cb = client_publish_timeout_cb;
    ERROR_CHECK(simple_battery_client_init(&m_bat_clients[0], 1));
    ERROR_CHECK(access_model_subscription_list_alloc(m_bat_clients[0].model_handle));
    

    //Initial Battery Server Model.
    m_bat_servers[0].set_unreliable_cb = bat_server_set_unreliable_cb;
    ERROR_CHECK(simple_battery_server_init(&m_bat_servers[0], 2));
    ERROR_CHECK(access_model_subscription_list_alloc(m_bat_servers[0].model_handle));
    

    //Initial Emergency Help Model.
    m_emh_clients[0].status_cb = emh_client_status_cb;
    m_emh_clients[0].timeout_cb = client_publish_timeout_cb;
    ERROR_CHECK(emergency_help_client_init(&m_emh_clients[0], 1));
    ERROR_CHECK(access_model_subscription_list_alloc(m_emh_clients[0].model_handle));

    m_emh_servers[0].set_unreliable_cb = emh_server_set_unreliable_cb;
    m_emh_servers[0].set_cb            = emh_server_set_cb;
    ERROR_CHECK(emergency_help_server_init(&m_emh_servers[0], 2));
    ERROR_CHECK(access_model_subscription_list_alloc(m_emh_servers[0].model_handle));
    
    
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
    }
}

static void battery_publish_timeout_handler(void *p_context)
{
    static uint8_t status;
    static uint32_t times = 0;
    static uint32_t node_addr = 0;

    if(!node_addr) node_addr = get_node_addr();

    times = times%3; //1:1.2:2.3:3......(min)
    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Times %d\n", times);  
    if(!times)
    {
        nrf_drv_saadc_sample_convert(0, &adc_value);
        
        status = simple_battery_client_set_unreliable(&m_bat_clients[0], adc_value, GROUP_MSG_REPEAT_COUNT, node_addr);
        if (status == NRF_SUCCESS)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Publish BAT value : %d to 0x%04x\n", adc_value, self_node_attr.destination_addr);
        }
    }
    times ++;
}

static void ch_election_timeout_handler(void *p_context)
{
    //ch = get_ch_node();
    ch_election_adv(ch.max_addr >> 8, ch.max_addr & 0xff, ch.max_priority);
}


static void i_am_ch_timeout_handler(void *p_context)
{
    i_am_ch_adv();
}

static void mn_ex_routing_timeout_handler(void *p_context)
{
    mn_routing_adv();
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

/**
 * Status :
 * 1 : Devide level Mode.       BS_relay_adv(uint8_t rc_level, uint8_t relay_tid)
 * 2 : CH election Mode.        ch_election_adv(uint8_t max_addr_H, uint8_t max_addr_L, uint8_t max_priority)
 * 3 : I am CH broadcast adv.   i_am_ch_adv(void)
 * 4 : MNs exchange routing.    mn_routing_adv(void)
 */
static void adv_tx_complete_cb(advertiser_t *p_adv, nrf_mesh_tx_token_t token, uint32_t timestamp)
{ 
    if(p_adv->p_packet != NULL)
    {
        //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Tx Done\n");
        //advertiser_packet_discard(p_adv, p_adv->p_packet);

        //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Status %d,Tx Complete, timestamp %d\n", adv_packet_status, timestamp);
        advertiser_packet_discard(p_adv, p_adv->p_packet);
        advertiser_disable(p_adv);
    
    }
}

static void adv_init(void)
{
    advertiser_instance_init(&m_advertiser, adv_tx_complete_cb, m_adv_buffer, ADVERTISER_BUF_SIZE);
    advertiser_interval_set(&m_advertiser, BEACON_INTERVAL);
}

//Use for bs devide level.    status : 1.
static void BS_relay_adv(uint8_t rc_level, uint8_t relay_tid)
{
    static bool first_flag = 0;       //First start have run @ch_election_adv().
    static uint8_t first_timer = 0;   //First start, 3min go to ch election. 1 times 10(s)
    adv_packet_status = 1;

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
        MAJOR_LOCATE_H , MAJOR_LOCATE_L,
        MINOR_LOCATE_H , MINOR_LOCATE_L,
        0x00,                                //Level
        0x00                                //Repeat tid
    };

    adv_data[28] = rc_level+1;
    adv_data[29] = relay_tid;

    self_node_attr.level = rc_level+1;

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "\nReceive BS ADV\nMAJOR 0x%02x%02x\nMINOR 0x%02x%02x\nLevel %d\nTid %d\n", adv_data[24], adv_data[25], adv_data[26], adv_data[27], adv_data[28], adv_data[29]);

    adv_packet_t *p_packet = advertiser_packet_alloc(&m_advertiser, sizeof(adv_data));
    if(p_packet){
        memcpy(p_packet->packet.payload, adv_data, sizeof(adv_data));

        p_packet->config.repeats = ADVERTISER_REPEAT_INFINITE;
        advertiser_packet_send(&m_advertiser, p_packet);
    }

    if(!first_flag && first_timer == BS_RELAY_TIMEOUT)
    {
        first_flag = 1;
        ch_election_start();
    }

    first_timer ++;
}

//Use for ch election.        status : 2.
static void ch_election_adv(uint8_t max_addr_H, uint8_t max_addr_L, uint8_t max_priority)
{
    //static uint16_t node_address = 0;
    static bool first_flag = 0;
    static uint8_t times = 0;
    adv_packet_status = 2;

    //Get self unicast address.
    //if(!node_address) node_address = get_node_addr();

    advertiser_enable(&m_advertiser);
    static uint8_t adv_data[] = 
    {
        0x02, 0x01, 0x06,
        0x1a, 0xff, 0x00, 0x59,
        0x02, 0x15,
        0x02, 0x56, 0xf6, 0xde, 0xde,     //CH_ADV_PREFIX
        0x00, 0x00,                       //max addr.         14.15
        0x00,                             //max priority.     16
        0xff, 0xff,                       //self addr         17.18
        0x00, 0x00,   
        0x00, 0x11, 0x22, 0x33,
        MAJOR_LOCATE_H, MAJOR_LOCATE_L,   
        MINOR_LOCATE_H, MINOR_LOCATE_L,
        0x88
    };
    
    //test
    if(!max_priority)
    {
        adv_data[14] = (uint8_t)(self_node_attr.self_addr>>8);
        adv_data[15] = (uint8_t)(self_node_attr.self_addr&0xFF);
        adv_data[16] = 0;
        adv_data[17] = (uint8_t)(self_node_attr.self_addr>>8);
        adv_data[18] = (uint8_t)(self_node_attr.self_addr&0xFF);

        first_flag = 1;
    }
    else
    {
        adv_data[14] = max_addr_H;
        adv_data[15] = max_addr_L;
        adv_data[16] = max_priority;
        adv_data[17] = (uint8_t)(self_node_attr.self_addr>>8);
        adv_data[18] = (uint8_t)(self_node_attr.self_addr&0xFF);
    }
    
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "\nCH Election ADV\nMAX Addr 0x%02x%02x\nMAX Priority %02d\nSelf Addr 0x%02x%02x\n", adv_data[14], adv_data[15], adv_data[16], adv_data[17], adv_data[18]);

    adv_packet_t *p_packet = advertiser_packet_alloc(&m_advertiser, sizeof(adv_data));
    if(p_packet){
        memcpy(p_packet->packet.payload, adv_data, sizeof(adv_data));

        p_packet->config.repeats = ADVERTISER_REPEAT_INFINITE;

        advertiser_packet_send(&m_advertiser, p_packet);
    }

    if(times == CH_ELECTION_TIMEOUT)
    {
        times = 0;
        app_timer_stop(m_ch_election_id);

        if(adv_data[17] == adv_data[14] && adv_data[18] == adv_data[15])
        {
            //i_am_ch_adv();
            self_node_attr.i_am_ch = 1;
            
            i_am_ch_start();
        };
    }

    times++;
}

//CH broadcast adv data.     status 3          
static void i_am_ch_adv(void)
{
    static uint8_t times = 0;
    //static uint16_t node_address = 0;
    
    adv_packet_status = 3;
    
    //if(!node_address) node_address = get_node_addr();

    advertiser_enable(&m_advertiser);
    static uint8_t adv_data[] = 
    {
        0x02, 0x01, 0x06,
        0x1a, 0xff, 0x00, 0xff,
        0x02, 0x15,
        0xf8, 0x44, 0x7e, 0x27,
        0x70, 0xcc, 0x64, 0xd8,
        0x35, 0xbb, 0x0c, 0xe0,
        MAJOR_LOCATE_H, MAJOR_LOCATE_L,   
        MINOR_LOCATE_H, MINOR_LOCATE_L,
        0x00, 0x00,                 //Self addr
        0x00, 0x00,                 //level
        0x00                        
    };

    adv_data[25] = (uint8_t)(self_node_attr.self_addr>>8);
    adv_data[26] = (uint8_t)(self_node_attr.self_addr&0xFF);
    adv_data[27] = self_node_attr.level;

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "\nI am CH\nSelf Addr 0x%02x%02x\nSelf level %d\n", adv_data[25], adv_data[26], adv_data[27]);

    adv_packet_t *p_packet = advertiser_packet_alloc(&m_advertiser, sizeof(adv_data));
    if(p_packet){
        memcpy(p_packet->packet.payload, adv_data, sizeof(adv_data));

        p_packet->config.repeats = ADVERTISER_REPEAT_INFINITE;

        advertiser_packet_send(&m_advertiser, p_packet);
    }

    if(times == I_AM_CH_TIMEOUT)
    {
        app_timer_stop(m_i_am_ch_id);
        times = 0;
        if(self_node_attr.level == 1)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "\nI AM CH\nSelf Addr 0x%04x publish destination addr 0x0000\n", self_node_attr.self_addr);
            //set publish addr.
            self_node_attr.destination_addr = 0x0000;
            set_model_publish_address(self_node_attr.destination_addr+2, m_bat_clients[0].model_handle);
            set_model_publish_address(self_node_attr.destination_addr+2, m_emh_clients[0].model_handle);
            bat_publish_start();
        }else
        {
            //set publish addr.
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "\nI AM CH\nSelf Addr 0x%04x publish destination addr 0x%04x\n", self_node_attr.self_addr, self_node_attr.destination_addr+2);
            set_model_publish_address(self_node_attr.destination_addr+2, m_bat_clients[0].model_handle);
            set_model_publish_address(self_node_attr.destination_addr+2, m_emh_clients[0].model_handle);
            
        }
    }else times ++;

}

//MNs change router.   status 4            
static void mn_routing_adv(void)
{
    adv_packet_status = 4;
    //static uint16_t node_address = 0;
    static uint8_t times = 0;
    dsm_handle_t add_addr_handle = DSM_HANDLE_INVALID;
    
    //if(!node_address) node_address = get_node_addr();
    
    advertiser_enable(&m_advertiser);
    static uint8_t adv_data[] = 
    {
        0x02, 0x01, 0x06,
        0x1a, 0xff, 0x00, 0xff,
        0x02, 0x15,
        0x38, 0x32, 0x89, 0xbf,
        0xde, 0xc2, 0x71, 0x84,
        0x4a, 0x47, 0x24, 0x82,
        MAJOR_LOCATE_H, MAJOR_LOCATE_L,   
        MINOR_LOCATE_H, MINOR_LOCATE_L,
        0x00, 0x00,                 //Self addr
        0x00, 0x00,                 //hop and level
        0x00                        //Destination addr can to bs flag.
    };

    adv_data[25] = (uint8_t)(self_node_attr.self_addr>>8);
    adv_data[26] = (uint8_t)(self_node_attr.self_addr&0xFF);
    adv_data[27] = self_node_attr.hop+1;
    adv_data[28] = self_node_attr.level;
    adv_data[29] = 1;

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "\nMNs Routing Exchange\nSelf Addr 0x%02x%02x\nHop %d\nSelf level %d\nDestination Flag\n", adv_data[25], adv_data[26], adv_data[27], adv_data[28], adv_data[29]);

    adv_packet_t *p_packet = advertiser_packet_alloc(&m_advertiser, sizeof(adv_data));
    if(p_packet){
        memcpy(p_packet->packet.payload, adv_data, sizeof(adv_data));

        p_packet->config.repeats = ADVERTISER_REPEAT_INFINITE;

        advertiser_packet_send(&m_advertiser, p_packet);
    }

    if(times == MN_ROUTING_ADV_TIMEOUT)
    {
        app_timer_stop(m_mn_ex_routing_id);
        times = 0;
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "\nI am MNs\nSelf Addr 0x%04x publish destination addr 0x%04x\n", self_node_attr.self_addr, self_node_attr.destination_addr);

        set_model_publish_address(self_node_attr.destination_addr+2, m_bat_clients[0].model_handle);
        set_model_publish_address(self_node_attr.destination_addr+2, m_emh_clients[0].model_handle);
        bat_publish_start();

    }else times ++;
}

static void adv_disable(void)
{
    advertiser_disable(&m_advertiser);
}


//**************                              HAL Init                  *************************/
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

    //Bat data trasnmit Timer.
    err_code = app_timer_create(&m_bat_publish_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_publish_timeout_handler);
    APP_ERROR_CHECK(err_code);

    //CH election process Timer.
    err_code = app_timer_create(&m_ch_election_id,
                                APP_TIMER_MODE_REPEATED,
                                ch_election_timeout_handler);
    APP_ERROR_CHECK(err_code);

    //I am CH Timer.(combine MNs exchange routin)
    err_code = app_timer_create(&m_i_am_ch_id,
                                APP_TIMER_MODE_REPEATED,
                                i_am_ch_timeout_handler);
    APP_ERROR_CHECK(err_code);

    //MNs exchange routing Timer. (combine i am ch timer)
    err_code = app_timer_create(&m_mn_ex_routing_id,
                                APP_TIMER_MODE_REPEATED,
                                mn_ex_routing_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

static void driver_init()
{
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


static void start(void)
{
    //static uint16_t node_address;

    rtt_input_enable(rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);
    ERROR_CHECK(mesh_stack_start());

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
        //Get self node addr.
        self_node_attr.self_addr = get_node_addr();
        provision_flag = 1;
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x\n", self_node_attr.self_addr);
    }

    const uint8_t *p_uuid = nrf_mesh_configure_device_uuid_get();
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Device UUID ", p_uuid, NRF_MESH_UUID_SIZE);

    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);

}

int main(void)
{
    initialize();
    execution_start(start);

    nrf_drv_saadc_sample_convert(0, &adc_value);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "ADC Value %d\n", adc_value);

    for (;;)
    {
        (void)sd_app_evt_wait();
    }
}