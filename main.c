/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a application that uses the Nordic UART service and FreeRTROS.
 *.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "peer_manager.h"
#include "ble_conn_state.h"
#include "nrf.h"
#include "nrf_drv_clock.h"
#include "nrf_temp.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_gpiote.h"


#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include <stdint.h>
#include <string.h>





#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                     "7dk29kshnsdc"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */
#define SLAVE_TYPE											'B'																					/**< The type slave, capital letter means it has a temp sensor*/ 
#define NORMAL_PRIORITY									0x0F														   					/**< The highest priority for the slave*/
#define LOW_PRIORITY										0x19																				/**< The start up priority*/

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                    	      /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             3                                           /**< Value of the RTC1 PRESCALER register. */  // Changed from 3
#define APP_TIMER_OP_QUEUE_SIZE         5                                           /**< Size of timer operation queues. */ // Changed from 4

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */
#define TEMP_UPDATE_INTERVAL						10000																				/**< Number of milliseconds the temperature should be calculated*/
#define ACK_WAIT_INTERVAL								5000																				/**< Number of milliseconds waiting for ack*/
#define RECONNECT_WAIT_INTERVAL					15000																					/**< Number of milliseconds waiting for a completed sending*/
#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */
							
#define ELEMENTS_IN_MY_DATA_STRUCT		  7
#define BLE_GAP_WHITELIST_ADDR_MAX_COUNT_EDITED 1

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

/* Common addresses definition for temperature sensor. */
#define LM75B_ADDR          (0x49U) 
#define LM75B_REG_TEMP      0x00U
#define LM75B_REG_CONF      0x01U
#define LM75B_REG_THYST     0x02U
#define LM75B_REG_TOS       0x03U

#define RELAY_GPIO_OUTPUT   11																											/**< Output pin for relay */

#define LED_STATE 		      2																												/**< STATE led, LED3 on board */
#define LED_RELAY     			3																												/**< Relay led, LED4 on board */				

/* Mode for LM75B. */
#define NORMAL_MODE 0U

APP_TIMER_DEF(m_temp_timer);
APP_TIMER_DEF(ack_timer);
APP_TIMER_DEF(reconnect_timer);

//static pm_peer_id_t m_peer_id;                             												  /**< Device reference handle to the current bonded central. */
static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */
void send_data(void);
bool send_ack(void);
bool waiting_ack = false;
static bool connected = false;

/* Variables for I2C temp sensor */
static volatile bool m_xfer_done = false;
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
static uint8_t m_sample = 0;
void update_state(void);
void thermostate(void);
void init_data(void);

typedef struct 
{
	uint8_t type;												/**< Type of slave device */
	uint8_t address;										/**< Address given by central */
	uint8_t ack;										/**< etc */		
	uint8_t state;										/**< etc */
	int8_t wanted_temp;												/**< Integer part of extern temp sensor on NRF52 */
	int8_t current_temp;										/**< Fractional part of extern temp semsor on NRF52 */
	uint8_t priority;										/**< The priority of the slave device */
}my_data;
	
my_data slave_data;

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}



/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function to print data to slaves
 * 
 *  
 */
void print_slave_data(void)
{	
		NRF_LOG_INFO("	Type:				%c\n\r",slave_data.type);
		NRF_LOG_INFO("	Address:			%d\n\r",slave_data.address);
		//NRF_LOG_INFO("	ack:				%d\n\r",slave_data.ack);
		NRF_LOG_INFO("	state:				%d\n\r",slave_data.state);
		NRF_LOG_INFO("	Wanted_temp:			%i\n\r",slave_data.wanted_temp);
		NRF_LOG_INFO("	Current_temp:			%i\n\r",slave_data.current_temp);
		NRF_LOG_INFO("	Priority:			%d\n\n\r",slave_data.priority);
 }


/**@brief Application handler
 */
void update_priority(void)
{
		if((slave_data.current_temp<slave_data.wanted_temp)&&(slave_data.wanted_temp - slave_data.current_temp)>= 2 )
		{
			if(NORMAL_PRIORITY != slave_data.priority)
			{
				slave_data.priority = NORMAL_PRIORITY;
				NRF_LOG_INFO("	New priority: NORMAL\r\n")
			}
			
		}else
		{
			
			if(LOW_PRIORITY != slave_data.priority)
			{
				slave_data.priority = LOW_PRIORITY;
				NRF_LOG_INFO("	New priority: LOW\r\n\n")
			}
				
		}			
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will store the data received from the Nordic UART BLE Service.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
	{	
					if('A' == p_data[0] || 'a' == p_data[0])
					{
						slave_data.address = p_data[1];
						app_timer_stop(reconnect_timer);
						if((slave_data.wanted_temp != p_data[4]) || ((p_data[3] != slave_data.state) && (0xFF != p_data[3]) ))
						{
							slave_data.wanted_temp = p_data[4];
							slave_data.state = p_data[3]; 
							update_state();
							update_priority();
							thermostate();
						}
						
						//Check if ack or data packet
						if(0 == p_data[2])
						{
							send_ack();	
							
						}else if(1== p_data[2])
						{
							NRF_LOG_INFO("	ack recieved \r\n\n");
							app_timer_stop(ack_timer);
							waiting_ack = false;
						}
				

					}else
					{
						NRF_LOG_INFO("	Recieved data from unknown source %c \n\n\r",p_data[0]);
					}
}


/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
		
    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
				NRF_LOG_INFO("\tsd_ble_gap_disconnect in on_conn_params_evt err_code:  %d \r\n",err_code);
				UNUSED_VARIABLE(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
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


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}




/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;
	
    switch (ble_adv_evt)
    {
				
				case BLE_ADV_EVT_DIRECTED:
            NRF_LOG_INFO("BLE_ADV_EVT_DIRECTED\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
            APP_ERROR_CHECK(err_code);
            break; 
				
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
				
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
				
				case BLE_ADV_EVT_PEER_ADDR_REQUEST:
						NRF_LOG_INFO("ADDR_request\n\r");

						break;
				
        default:
            break;
    }
}



/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

	
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
		
						NRF_LOG_INFO("	BLE_GAP_EVT_CONNECTED \n\r ");
						connected = true;
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break; // BLE_GAP_EVT_CONNECTED
				
				case BLE_GAP_EVT_ADV_REPORT:
				{ 
            
                    
					}break; // BLE_GAP_EVT_ADV_REPORT
									
        case BLE_GAP_EVT_DISCONNECTED:
						
						NRF_LOG_INFO("	BLE_GAP_EVT_DISCONNECTED \n\r ");
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
				
						err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
						if(0 != err_code)
						{
								NRF_LOG_INFO("ble_advertising_start in BLE_GAP_EVT_DISCONNECTED err_code; 0x%x \n\r ",err_code);
						}
						init_data();
						
						connected=false;
				
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
						NRF_LOG_INFO("	BLE_GAP_EVT_SEC_PARAMS_REQUEST \n\r ");
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
						NRF_LOG_INFO("	BLE_GATTS_EVT_SYS_ATTR_MISSING \n\r ");
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_SYS_ATTR_MISSING

        case BLE_GATTC_EVT_TIMEOUT:				
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
						NRF_LOG_INFO("	BLE_GATTC_EVT_TIMEOUT \n\r ");
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
						NRF_LOG_INFO("	BLE_GATTS_EVT_TIMEOUT \n\r ");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
						NRF_LOG_INFO("	BLE_EVT_USER_MEM_REQUEST \n\r ");
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
						NRF_LOG_INFO("	BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST \n\r ");
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
						NRF_LOG_INFO("	BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST \n\r ");
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

	
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
	#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
	#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
			
						NRF_LOG_INFO("	disconnected_bps_Event");
			
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
						
						err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
						APP_ERROR_CHECK(err_code);
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist();
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;
				case BSP_EVENT_KEY_3:
            err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
						APP_ERROR_CHECK(err_code);
            break;
        default:
            break;
    }
}






/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advdata_t          advdata;
    ble_advdata_t          scanrsp;
    ble_adv_modes_config_t options;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
		options.ble_adv_whitelist_enabled =true;
		options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;
		

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for turning on or off reley.
 */
void update_state()
{
	if(100 == slave_data.state)
	{
		bsp_board_led_on(LED_STATE);
		NRF_LOG_INFO("	STATE ON ");
	}
	else
	{
		bsp_board_led_off(LED_STATE);
		NRF_LOG_INFO("	STATE OFF ");
	}
}


/**@brief Thermostate function
 */
void thermostate(void)
{
	if(slave_data.wanted_temp > slave_data.current_temp && 100 == slave_data.state)
	{
		NRF_LOG_INFO("	OVEN ON\r\n ");
		bsp_board_led_on(LED_RELAY);
		nrf_gpio_pin_set(RELAY_GPIO_OUTPUT);
	}
	else
	{
		NRF_LOG_INFO("	OVEN OFF\r\n ");
		bsp_board_led_off(LED_RELAY);
		nrf_gpio_pin_clear(RELAY_GPIO_OUTPUT);
	}
}


/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for setting initial data values
 */
void init_data()
{
	
		slave_data.type 	  = SLAVE_TYPE;
		slave_data.address 	= 0xFF;
		slave_data.ack  = 0;
		slave_data.state = 100;
		slave_data.wanted_temp = 0xFF;
		slave_data.current_temp = 0;
		slave_data.priority = LOW_PRIORITY;
}


/**@brief Function to send data over Nordic Uart serial
 * @param[out] bool true or false, sending successful or not
 */
void send_data(void)
{
	if(true == connected )
	{
		uint8_t data[7];
		uint32_t err_code;
		uint32_t err_code_timer;
		
		data[0] = slave_data.type;
		data[1] = slave_data.address;
		data[2] = 0;
		data[3] = slave_data.state;
		//data[4] = slave_data.wanted_temp;
		data[5] = slave_data.current_temp;
		data[6] = slave_data.priority;
		
		if(false == waiting_ack)
		{
				err_code = ble_nus_string_send(&m_nus,data,ELEMENTS_IN_MY_DATA_STRUCT);
				waiting_ack = true;
			

			if(err_code == NRF_SUCCESS )
			{
					NRF_LOG_INFO("	sending complete \n\r");
					err_code_timer = app_timer_start(ack_timer, 
																	 APP_TIMER_TICKS(ACK_WAIT_INTERVAL, 
																	 APP_TIMER_PRESCALER),
																					NULL);
				app_timer_stop(reconnect_timer);
				if(err_code_timer != NRF_SUCCESS)
					NRF_LOG_INFO("\tapp_timer_error ack_timer; 0x%x  \n\r",err_code_timer);
				
					
			}
			else
				{
					NRF_LOG_INFO("	sending not complete, error: 0x%x  %d \n\r",err_code,err_code);
					err_code = app_timer_start(reconnect_timer, 
																	 APP_TIMER_TICKS(RECONNECT_WAIT_INTERVAL, 
																	 APP_TIMER_PRESCALER),
																					NULL);
				}
		}
		else NRF_LOG_INFO("	sending not complete, waiting on ack  \n\r");
	}
}

/**@brief Function to send ack over Nordic Uart serial
 * @param[out] bool true or false, sending successful not not
 */
bool send_ack(void)
{
	
	uint8_t data[ELEMENTS_IN_MY_DATA_STRUCT];
  uint32_t err_code;
	
	data[0] = slave_data.type;
	data[1] = slave_data.address;
	data[2] = 1;
	data[3] = slave_data.state;
	//data[4] = slave_data.wanted_temp;
	data[5] = slave_data.current_temp;
	data[6] = slave_data.priority;
	
	err_code = ble_nus_string_send(&m_nus,data,ELEMENTS_IN_MY_DATA_STRUCT);
	
	if(err_code == NRF_SUCCESS )
	{
			NRF_LOG_INFO("	ack sent \n\n\r");
			app_timer_stop(reconnect_timer);
		
			return true;
	}
	else
		{
			NRF_LOG_INFO("	ack failed to send: 0x%02x \n\n\r",err_code);
			return false;
		}
}




/**
 * @brief Function for setting active mode on MMA7660 accelerometer.
 */
void LM75B_set_mode(void)
{
    ret_code_t err_code;

    /* Writing to LM75B_REG_CONF "0" set temperature sensor in NORMAL mode. */
    uint8_t reg[2] = {LM75B_REG_CONF, NORMAL_MODE};
    err_code = nrf_drv_twi_tx(&m_twi, LM75B_ADDR, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    /* Writing to pointer byte. */
    reg[0] = LM75B_REG_TEMP;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, LM75B_ADDR, reg, 1, false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
}


/**
 * @brief Function for handling data from temperature sensor.
 *
 * @param[in] temp          Temperature in Celsius degrees read from sensor.
 */
__STATIC_INLINE void data_handler(uint8_t temp)
{
		//NRF_LOG_INFO("KJ�RER DENNE_2????");

		
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                data_handler(m_sample);
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}


/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_lm75b_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_lm75b_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}


/**
 * @brief Function for reading data from temperature sensor.
 */
static void read_sensor_data()
{
    m_xfer_done = false;

    /* Read 1 byte from the specified address - skip 3 bits dedicated for fractional part of temperature. */
    ret_code_t err_code = nrf_drv_twi_rx(&m_twi, LM75B_ADDR, &m_sample, sizeof(m_sample));
    APP_ERROR_CHECK(err_code);
}


/**@brief  Timeout handler for the repeated timer
 */
static void timer_handler(void * p_context)
{	
		static volatile bool updated_temp = false;
	
		if(0xFF == slave_data.address && true == connected )
		{
			NRF_LOG_INFO("	What is my address ? \r\n");
			send_data();
		}
			
		NRF_LOG_INFO("\tread_sensor_data: %i \r\n",m_sample);
		
		if(m_sample != slave_data.current_temp || false == updated_temp)
		{

			NRF_LOG_INFO("	New temp: %d \r\n",m_sample);
			slave_data.current_temp = m_sample;
			thermostate();
			update_priority();
			
			if(slave_data.current_temp != 0  )
			{
				
				NRF_LOG_INFO("	Set update_temp to true: %d \r\n",slave_data.current_temp);
				updated_temp = true;
			}	
			send_data();	
		}
		
		print_slave_data();
}

/**@brief  Timeout handler for the repeated timer
 */
static void ack_timer_handler(void * p_context)
{	
		send_data();
		NRF_LOG_INFO("	Did not receive any ack, resending data\r\n");
}

static void reconnect_timer_handler(void * p_context)
{		
	static volatile uint8_t cnt =0;
	cnt++;
	if(slave_data.address == 255 || cnt > 100)
	{
	
				NRF_LOG_INFO("	Connection problem, resetting device \r\n");
				APP_ERROR_CHECK(1);			
	}
}

/**@brief Create timers.
 */
static void create_timers()
{   
    uint32_t err_code;
	
    err_code = app_timer_create(&m_temp_timer, 
																APP_TIMER_MODE_REPEATED,
																timer_handler);

    APP_ERROR_CHECK(err_code);
		err_code = app_timer_create(&ack_timer, 
																APP_TIMER_MODE_SINGLE_SHOT,
																ack_timer_handler);
	
		APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_create(&reconnect_timer, 
																APP_TIMER_MODE_SINGLE_SHOT,
																reconnect_timer_handler);
		APP_ERROR_CHECK(err_code);
}


/**@brief Application main function.
 */
int main(void)
{	
		
    uint32_t err_code;
    bool erase_bonds;
		

    // Initialize
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
		create_timers();
		err_code = app_timer_start(m_temp_timer, 
															 APP_TIMER_TICKS(TEMP_UPDATE_INTERVAL, 
															 APP_TIMER_PRESCALER),NULL);

																	
		twi_init();		
	  LM75B_set_mode();
    buttons_leds_init(&erase_bonds);
	
		bsp_board_leds_init();
		err_code = NRF_LOG_INIT(NULL);
		APP_ERROR_CHECK(err_code);
	
		if (erase_bonds == true)
    {
        NRF_LOG_DEBUG("Bonds erased!\r\n");
    }
		nrf_drv_gpiote_init();
		APP_ERROR_CHECK(err_code);
		NRF_LOG_INFO("GPIO_DRIVER_INIT: %i \r\n",err_code);
		nrf_gpio_pin_dir_set(RELAY_GPIO_OUTPUT,NRF_GPIO_PIN_DIR_OUTPUT);					//GPIO_PIN_CNF_DIR_Output
		
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
		init_data();
		update_state();
		thermostate();
		
		
		
    NRF_LOG_INFO("	UART Start!!!\r\n");
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
		if(err_code != 0)
			NRF_LOG_INFO("\tble_advertising_start err_code: %d \r\n",err_code);
		
    UNUSED_VARIABLE(err_code);

    // Enter main loop.
    for (;;)
    {							
				 do
        {
           power_manage();	
					 NRF_LOG_FLUSH(); 
        }while (m_xfer_done == false);

        read_sensor_data();
        
    }
}


/**
 * @}
 */
