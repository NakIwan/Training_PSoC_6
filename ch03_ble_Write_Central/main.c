#include "cybsp.h"
#include "cyhal.h"
#include "cy_retarget_io.h"

/* Library for malloc and free */
#include "stdlib.h"

/* FreeRTOS */
#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>

/* btstack */
#include "wiced_bt_stack.h"

/* App utilities */
#include "app_bt_utils.h"

/* Include header files from BT configurator */
#include "cycfg_bt_settings.h"
#include "cycfg_gap.h"
#include "cycfg_gatt_db.h"
#include <string.h>


/*******************************************************************
 * Macros to assist development of the exercises
 ******************************************************************/
#ifndef CYBSP_USER_LED2
#define CYBSP_USER_LED2 P7_1
#endif

#define TASK_STACK_SIZE (4096u)
#define	TASK_PRIORITY 	(5u)

#define APP_TIMEOUT_LED_BLINK          (100)

#define BAUD_RATE       9600
#define UART_DELAY      10u
#define RX_BUF_SIZE     32
#define TX_BUF_SIZE     32
#define RX_US100 P10_0
#define TX_US100 P10_1


/* Typdef for function used to free allocated buffer to stack */
typedef void (*pfn_free_buffer_t)(uint8_t *);

/*******************************************************************
 * Function Prototypes
 ******************************************************************/
/* Callback function for Bluetooth stack management type events */
static wiced_bt_dev_status_t app_bt_management_callback             (wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);

/* GATT Event Callback and Handler Functions */
static wiced_bt_gatt_status_t app_bt_gatt_event_callback            (wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data);

static wiced_bt_gatt_status_t app_bt_connect_event_handler          (wiced_bt_gatt_connection_status_t *p_conn_status);
static wiced_bt_gatt_status_t app_bt_server_event_handler           (wiced_bt_gatt_event_data_t *p_data);

static wiced_bt_gatt_status_t app_bt_write_handler                  (wiced_bt_gatt_event_data_t *p_data);
static wiced_bt_gatt_status_t app_bt_gatt_req_read_handler          (uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
                                                                    wiced_bt_gatt_read_t *p_read_req, uint16_t len_requested);
static wiced_bt_gatt_status_t app_bt_gatt_req_read_multi_handler    (uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
                                                                    wiced_bt_gatt_read_multiple_req_t *p_read_req, uint16_t len_requested);
static wiced_bt_gatt_status_t app_bt_gatt_req_read_by_type_handler  (uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
                                                                    wiced_bt_gatt_read_by_type_t *p_read_req, uint16_t len_requested);

/* Helper functions to find GATT database handles and allocate/free buffers for GATT operations */
static gatt_db_lookup_table_t 	*app_bt_find_by_handle(uint16_t handle);
static uint8_t 					*app_bt_alloc_buffer(uint16_t len);
static void 					app_bt_free_buffer(uint8_t *p_data);

/*******************************************************************
 * Global/Static Variables
 ******************************************************************/

/* Enable RTOS aware debugging in OpenOCD */
volatile int uxTopUsedPriority;

cyhal_pwm_t PWM_obj;

TaskHandle_t NotifyTaskHandle = NULL;
TaskHandle_t NotifyTaskHandle1 = NULL;
TimerHandle_t timer_led_blink;

uint8_t flag_US100;
uint8_t led_blink_count;

uint8_t      tx_buf[TX_BUF_SIZE];
uint8_t      rx_buf[RX_BUF_SIZE];
uint8_t      read_data[RX_BUF_SIZE];

uint16_t Distance;
uint16_t connection_id = 0;

size_t       tx_length = TX_BUF_SIZE;
size_t       rx_length = RX_BUF_SIZE;

const cyhal_uart_cfg_t uart_config =
{
	.data_bits = 8,
	.stop_bits = 1,
	.parity = CYHAL_UART_PARITY_NONE,
	.rx_buffer = rx_buf,
	.rx_buffer_size = RX_BUF_SIZE
};

void app_bt_timeout_led_blink(TimerHandle_t timer_handle)
{
    static wiced_bool_t led_on = WICED_TRUE;
    if (led_on)
    {
        cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
        if (--led_blink_count)
        {
            led_on = WICED_FALSE;
            xTimerStart(timer_led_blink, 0);
        }
    }
    else
    {
        led_on = WICED_TRUE;
        cyhal_gpio_write(CYBSP_USER_LED , CYBSP_LED_STATE_ON);
        xTimerStart(timer_led_blink, 0);
    }
}

void app_bt_led_blink(uint8_t num_of_blinks)
{
    if (num_of_blinks)
    {
    	for (int i = 0; i <num_of_blinks; i++){
            led_blink_count = num_of_blinks;
            cyhal_gpio_write(CYBSP_USER_LED , CYBSP_LED_STATE_ON);
            xTimerStart(timer_led_blink, 0);
    	}
    }
}

static void String_task(void * arg){

    cyhal_uart_t uart_obj;
    uint32_t     actualbaud;
	char buffer_text[24];

    cyhal_uart_init(&uart_obj, TX_US100, RX_US100, NC, NC, NULL, &uart_config);

    cyhal_uart_set_baud(&uart_obj, BAUD_RATE, &actualbaud);

	while(1){

		if (flag_US100){
			if (cyhal_uart_putc(&uart_obj, 0X55) == CY_RSLT_SUCCESS)
	    	{
	    		rx_length = 3;
	            cyhal_uart_read(&uart_obj, rx_buf, &rx_length);
	            Distance = (uint16_t)rx_buf[0] << 8 | rx_buf[1];
	            printf("US100 = %d mm\r\n",Distance);
	    	}
		}
		if(connection_id != 0){
    		if(app_psoc_string_client_char_config[0] & GATT_CLIENT_CONFIG_NOTIFICATION){
				sprintf(buffer_text,"US100 = %d", Distance);
				wiced_bt_gatt_server_send_notification( connection_id,
														HDLC_PSOC_STRING_VALUE,
														strlen(buffer_text),
														(uint8_t *)buffer_text,
														NULL);
    		}
		}
		vTaskDelay(50);
	}
}

int main(void)
{
    cy_rslt_t result ;

    /* Initialize the board support package */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,\
                        CY_RETARGET_IO_BAUDRATE);

    /* Initialize LED Pin */
    cyhal_gpio_init(CYBSP_USER_LED,CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    printf("**********Application Start*****************\n");

    result = cyhal_pwm_init(&PWM_obj, CYBSP_USER_LED2, NULL);
    cyhal_pwm_start(&PWM_obj);

    /* Configure platform specific settings for the BT device */
       cybt_platform_config_init(&cybsp_bt_platform_cfg);

    /* Initialize stack and register the callback function */
    wiced_bt_stack_init (app_bt_management_callback, &wiced_bt_cfg_settings);

    xTaskCreate (String_task, "StringTask", TASK_STACK_SIZE, NULL, TASK_PRIORITY, &NotifyTaskHandle1);

    timer_led_blink = xTimerCreate("led_timer",
                                    pdMS_TO_TICKS(APP_TIMEOUT_LED_BLINK),
                                    pdFALSE,
                                    NULL,
                                    app_bt_timeout_led_blink);

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler() ;

    /* Should never get here */
    CY_ASSERT(0) ;
}


/*******************************************************************************
* Function Name: wiced_bt_dev_status_t app_bt_management_callback(
* 					wiced_bt_management_evt_t event,
* 					wiced_bt_management_evt_data_t *p_event_data )
********************************************************************************/
static wiced_result_t app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
	/* Start in error state so that any unimplemented states will return error */
    wiced_result_t result = WICED_BT_ERROR;
    wiced_bt_device_address_t bda = {0};

    printf("Bluetooth Management Event: 0x%x %s\n", event, get_bt_event_name(event));

    switch( event )
    {
		case BTM_ENABLED_EVT:								// Bluetooth Controller and Host Stack Enabled
			if( WICED_BT_SUCCESS == p_event_data->enabled.status )
			{
				printf( "Bluetooth Enabled\n" );

				/* Set the local BDA from the value in the configurator and print it */
				wiced_bt_set_local_bdaddr((uint8_t *)cy_bt_device_address, BLE_ADDR_PUBLIC);
				wiced_bt_dev_read_local_addr( bda );
				printf( "Local Bluetooth Device Address: ");
				print_bd_address(bda);

				/* Register GATT callback and initialize database*/
				wiced_bt_gatt_register( app_bt_gatt_event_callback );
				wiced_bt_gatt_db_init( gatt_database, gatt_database_len, NULL );

				/* Disable pairing */
				wiced_bt_set_pairable_mode( WICED_FALSE, WICED_FALSE );

				/* Set advertisement packet and begin advertising */
				wiced_bt_ble_set_raw_advertisement_data(CY_BT_ADV_PACKET_DATA_SIZE,
				                                        cy_bt_adv_packet_data);
				wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );


	            result = WICED_BT_SUCCESS;
			}
			else
			{
				printf( "Failed to initialize Bluetooth controller and stack\n" );
			}
			break;

		case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT: 		// IO capabilities request
			result = WICED_BT_SUCCESS;
			break;

		case BTM_PAIRING_COMPLETE_EVT: 							// Pairing Complete event
			result = WICED_BT_SUCCESS;
			break;

		case BTM_ENCRYPTION_STATUS_EVT: 						// Encryption Status Event
			result = WICED_BT_SUCCESS;
			break;

		case BTM_SECURITY_REQUEST_EVT: 							// Security access
			result = WICED_BT_SUCCESS;
			break;

		case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT: 			// Save link keys with app
			result = WICED_BT_SUCCESS;
			break;

		case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT: 			// Retrieve saved link keys
            /* This must return WICED_BT_ERROR if bonding information is not stored in EEPROM */
			result = WICED_BT_ERROR;
			break;

		case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT: 				// Save keys to NVRAM
			result = WICED_BT_SUCCESS;
			break;

		case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT: 				// Read keys from NVRAM
            /* This should return WICED_BT_SUCCESS if not using privacy. If RPA is enabled but keys are not
               stored in EEPROM, this must return WICED_BT_ERROR so that the stack will generate new privacy keys */
			result = WICED_BT_ERROR;
			break;

		case BTM_BLE_SCAN_STATE_CHANGED_EVT: 					// Scan State Change
			result = WICED_BT_SUCCESS;
			break;

		case BTM_BLE_ADVERT_STATE_CHANGED_EVT:					// Advertising State Change
            printf("Advertisement State Change: %s\n", get_bt_advert_mode_name(p_event_data->ble_advert_state_changed));
            result = WICED_BT_SUCCESS;
            if (p_event_data->ble_advert_state_changed == BTM_BLE_ADVERT_OFF){
            	if(connection_id == 0){
            		cyhal_pwm_set_duty_cycle(&PWM_obj, 100, 2);
            	}
            	else { //connected
            		cyhal_pwm_set_duty_cycle(&PWM_obj, 0, 2);
            	}
            }
            else{
            		cyhal_pwm_set_duty_cycle(&PWM_obj, 50, 2);
            }
			break;

		default:
			break;
    }

    return result;
}


/*******************************************************************************
* Function Name: wiced_bt_gatt_status_t app_bt_gatt_event_callback(
* 					wiced_bt_gatt_evt_t event,
* 					wiced_bt_gatt_event_data_t *p_data )
********************************************************************************/
static wiced_bt_gatt_status_t app_bt_gatt_event_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data )
{
	/* Start in error state so that any unimplemented states will return error */
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    /* Call the appropriate callback function based on the GATT event type, and pass the relevant event
     * parameters to the callback function */
    switch (event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        status = app_bt_connect_event_handler(&p_event_data->connection_status);
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        status = app_bt_server_event_handler(p_event_data);
        break;

    case GATT_GET_RESPONSE_BUFFER_EVT: /* GATT buffer request, typically sized to max of bearer mtu - 1 */
        p_event_data->buffer_request.buffer.p_app_rsp_buffer = app_bt_alloc_buffer(p_event_data->buffer_request.len_requested);
        p_event_data->buffer_request.buffer.p_app_ctxt = (void *)app_bt_free_buffer;
        status = WICED_BT_GATT_SUCCESS;
        break;

    case GATT_APP_BUFFER_TRANSMITTED_EVT: /* GATT buffer transmitted event,  check \ref wiced_bt_gatt_buffer_transmitted_t*/
        {
            pfn_free_buffer_t pfn_free = (pfn_free_buffer_t)p_event_data->buffer_xmitted.p_app_ctxt;

            /* If the buffer is dynamic, the context will point to a function to free it. */
            if (pfn_free)
            {
                pfn_free(p_event_data->buffer_xmitted.p_app_data);
            }
            status = WICED_BT_GATT_SUCCESS;
        }
        break;

    default:
    	printf( "Unhandled GATT Event: 0x%x (%d)\n", event, event );
        status = WICED_BT_GATT_SUCCESS;
        break;
    }

    return status;
}


/*******************************************************************************
 * Function Name: app_bt_connect_event_handler
 *
 * Handles GATT connection status changes.
 *
 * Param:	p_conn_status  Pointer to data that has connection details
 * Return:	wiced_bt_gatt_status_t
 * See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
*******************************************************************************/
static wiced_bt_gatt_status_t app_bt_connect_event_handler(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    if (NULL != p_conn_status)
    {
        if (p_conn_status->connected)
        {
           	printf("GATT_CONNECTION_STATUS_EVT: Connect BDA ");
           	print_bd_address(p_conn_status->bd_addr);
			printf("Connection ID %d\n", p_conn_status->conn_id );
			connection_id = p_conn_status->conn_id;
			/* Handle the connection */
        }
        else
        {
            printf("Disconnected : BDA " );
            print_bd_address(p_conn_status->bd_addr);
            printf("Connection ID '%d', Reason '%s'\n", p_conn_status->conn_id, get_bt_gatt_disconn_reason_name(p_conn_status->reason) );
            connection_id = 0;
            flag_US100 = false;
			/* Handle the disconnection */

			/* Restart the advertisements */
			wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
        }

        status = WICED_BT_GATT_SUCCESS;
    }

    return status;
}


/*******************************************************************************
 * Function Name: app_bt_server_event_handler
 *
 * Invoked when GATT_ATTRIBUTE_REQUEST_EVT occurs in GATT Event callback.
 *
 * Param:	p_data   				Pointer to BLE GATT request data
 * Return:	wiced_bt_gatt_status_t  BLE GATT status
*******************************************************************************/
static wiced_bt_gatt_status_t app_bt_server_event_handler(wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
    wiced_bt_gatt_attribute_request_t   *p_att_req = &p_data->attribute_request;

    switch (p_att_req->opcode)
    {

		case GATT_REQ_READ: /* Attribute read notification (attribute value internally read from GATT database) */
		case GATT_REQ_READ_BLOB:
			status = app_bt_gatt_req_read_handler(p_att_req->conn_id, p_att_req->opcode,
																	   &p_att_req->data.read_req, p_att_req->len_requested);
			break;

		case GATT_REQ_READ_BY_TYPE:
			status = app_bt_gatt_req_read_by_type_handler(p_att_req->conn_id, p_att_req->opcode,
																   &p_att_req->data.read_by_type, p_att_req->len_requested);
			break;

		case GATT_REQ_READ_MULTI:
		case GATT_REQ_READ_MULTI_VAR_LENGTH:
			status = app_bt_gatt_req_read_multi_handler(p_att_req->conn_id, p_att_req->opcode,
															  &p_att_req->data.read_multiple_req, p_att_req->len_requested);
			break;

		case GATT_REQ_WRITE:
		case GATT_CMD_WRITE:
		case GATT_CMD_SIGNED_WRITE:
			status = app_bt_write_handler(p_data);
			if ((p_att_req->opcode == GATT_REQ_WRITE) && (status == WICED_BT_GATT_SUCCESS))
			{
				wiced_bt_gatt_write_req_t *p_write_request = &p_att_req->data.write_req;
				wiced_bt_gatt_server_send_write_rsp(p_att_req->conn_id, p_att_req->opcode, p_write_request->handle);
			}
			break;

		case GATT_REQ_PREPARE_WRITE:
			status = WICED_BT_GATT_SUCCESS;
			break;

		case GATT_REQ_EXECUTE_WRITE:
			wiced_bt_gatt_server_send_execute_write_rsp(p_att_req->conn_id, p_att_req->opcode);
			status = WICED_BT_GATT_SUCCESS;
			break;

		case GATT_REQ_MTU:
			/* Application calls wiced_bt_gatt_server_send_mtu_rsp() with the desired mtu */
			status = wiced_bt_gatt_server_send_mtu_rsp(p_att_req->conn_id,
													   p_att_req->data.remote_mtu,
													   wiced_bt_cfg_settings.p_ble_cfg->ble_max_rx_pdu_size);
			break;

		case GATT_HANDLE_VALUE_CONF: /* Value confirmation */
			break;

		case GATT_HANDLE_VALUE_NOTIF:
			break;

		default:
	    	printf( "Unhandled GATT Server Event: 0x%x (%d)\n", p_att_req->opcode, p_att_req->opcode );
			break;
    }

    return status;
}


/*******************************************************************************
 * Function Name: app_bt_write_handler
 *
 * Invoked when GATTS_REQ_TYPE_WRITE is received from the
 * client device. Handles "Write Requests" received from Client device.
 *
 * Param:	p_write_req   			Pointer to BLE GATT write request
 * Return:	wiced_bt_gatt_status_t  BLE GATT status
*******************************************************************************/
static wiced_bt_gatt_status_t app_bt_write_handler(wiced_bt_gatt_event_data_t *p_data)
{
	wiced_bt_gatt_write_req_t *p_write_req = &p_data->attribute_request.data.write_req;;

    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

     for (int i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
     {
         /* Check for a matching handle entry */
         if (app_gatt_db_ext_attr_tbl[i].handle == p_write_req->handle)
         {
             /* Detected a matching handle in the external lookup table */
             if (app_gatt_db_ext_attr_tbl[i].max_len >= p_write_req->val_len)
             {
                 /* Value fits within the supplied buffer; copy over the value */
                 app_gatt_db_ext_attr_tbl[i].cur_len = p_write_req->val_len;
                 memset(app_gatt_db_ext_attr_tbl[i].p_data, 0x00, app_gatt_db_ext_attr_tbl[i].max_len);
                 memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_write_req->p_val, app_gatt_db_ext_attr_tbl[i].cur_len);

                 if (memcmp(app_gatt_db_ext_attr_tbl[i].p_data, p_write_req->p_val, app_gatt_db_ext_attr_tbl[i].cur_len) == 0)
                 {
                	 status = WICED_BT_GATT_SUCCESS;
                 }

                 switch ( p_write_req->handle )
                 {
                 	 // Add action when specified handle is written
                 	 case HDLC_PSOC_LED_VALUE:

                  		app_bt_led_blink(app_psoc_led[0]);
                  		printf("%d\r\n",app_psoc_led[0]);

                 		 for (int i = 0; i < app_psoc_led_len; i++){
                      		printf("Data %d = %d\r\n",i ,app_psoc_led[i]);
                 		 }

                  		break;

                 	 case HDLC_PSOC_STRING_VALUE:
                 		 char *ptr = app_psoc_string;
                 		 printf("Data string = %s\r\n",ptr);

                 		 if(strcmp(app_psoc_string, "US100") == 0) flag_US100 = true;
                 		 else flag_US100 = false;

                  		break;
                 }
             }
             else
             {
                 /* Value to write will not fit within the table */
            	 status = WICED_BT_GATT_INVALID_ATTR_LEN;
                 printf("Invalid attribute length during GATT write\n");
             }
             break;
         }
     }
     if (WICED_BT_GATT_SUCCESS != status)
     {
         printf("GATT write failed: %d\n", status);
     }

     return status;
}


/*******************************************************************************
 * Function Name: app_bt_gatt_req_read_handler
 *
 * This Function handles GATT read and read blob events
 *
 * Params: 	conn_id       			Connection ID
 * 			opcode        			BLE GATT request type opcode
 * 			p_read_req    			Pointer to read request containing the handle to read
 * 			len_requested 			Length of data requested
 * Return: 	wiced_bt_gatt_status_t  BLE GATT status
*******************************************************************************/
static wiced_bt_gatt_status_t app_bt_gatt_req_read_handler(uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
                                                              wiced_bt_gatt_read_t *p_read_req, uint16_t len_requested)
{
    gatt_db_lookup_table_t *puAttribute;
    uint16_t attr_len_to_copy, to_send;
    uint8_t *from;

    if ((puAttribute = app_bt_find_by_handle(p_read_req->handle)) == NULL)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle, WICED_BT_GATT_INVALID_HANDLE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    attr_len_to_copy = puAttribute->cur_len;

    if (p_read_req->offset >= puAttribute->cur_len)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle, WICED_BT_GATT_INVALID_OFFSET);
        return WICED_BT_GATT_INVALID_OFFSET;
    }

	switch ( p_read_req->handle )
	{
		// Add action when specified handle is read
		case HDLC_PSOC_LED_VALUE:
			printf( "LED is %s\r\n", app_psoc_led[0] ? "ON" : "OFF" );
			break;
	}

    to_send = MIN(len_requested, attr_len_to_copy - p_read_req->offset);
    from = puAttribute->p_data + p_read_req->offset;
    return wiced_bt_gatt_server_send_read_handle_rsp(conn_id, opcode, to_send, from, NULL); /* No need for context, as buff not allocated */
}

/*******************************************************************************
 * Function Name: app_bt_gatt_req_read_by_type_handler
 *
 * Process read-by-type request from peer device
 *
 * Params:	conn_id       			Connection ID
 * 			opcode        			BLE GATT request type opcode
 * 			p_read_req    			Pointer to read request containing the handle to read
 * 			len_requested 			Length of data requested
 * Return:	wiced_bt_gatt_status_t	BLE GATT status
*******************************************************************************/
static wiced_bt_gatt_status_t app_bt_gatt_req_read_by_type_handler(uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
                                                       wiced_bt_gatt_read_by_type_t *p_read_req, uint16_t len_requested)
{
    gatt_db_lookup_table_t *puAttribute;
    uint16_t attr_handle = p_read_req->s_handle;
    uint8_t *p_rsp = app_bt_alloc_buffer(len_requested);
    uint8_t pair_len = 0;
    int used = 0;

    if (p_rsp == NULL)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, attr_handle, WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INSUF_RESOURCE;
    }

    /* Read by type returns all attributes of the specified type, between the start and end handles */
    while (WICED_TRUE)
    {
        attr_handle = wiced_bt_gatt_find_handle_by_type(attr_handle, p_read_req->e_handle, &p_read_req->uuid);

        if (attr_handle == 0)
            break;

        if ((puAttribute = app_bt_find_by_handle(attr_handle)) == NULL)
        {
            wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->s_handle, WICED_BT_GATT_ERR_UNLIKELY);
            app_bt_free_buffer(p_rsp);
            return WICED_BT_GATT_INVALID_HANDLE;
        }

        {
            int filled = wiced_bt_gatt_put_read_by_type_rsp_in_stream(p_rsp + used, len_requested - used, &pair_len,
                                                                attr_handle, puAttribute->cur_len, puAttribute->p_data);
            if (filled == 0)
            {
                break;
            }
            used += filled;
        }

        /* Increment starting handle for next search to one past current */
        attr_handle++;
    }

    if (used == 0)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->s_handle, WICED_BT_GATT_INVALID_HANDLE);
        app_bt_free_buffer(p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

	switch ( p_read_req->s_handle )
	{
		// Add action when specified handle is read
		case HDLC_PSOC_LED_VALUE:
			printf( "LED is %s\r\n", app_psoc_led[0] ? "ON" : "OFF" );
			break;
	}

    /* Send the response */
    wiced_bt_gatt_server_send_read_by_type_rsp(conn_id, opcode, pair_len, used, p_rsp, (void *)app_bt_free_buffer);

    return WICED_BT_GATT_SUCCESS;
}

/*******************************************************************************
 * Function Name: app_bt_gatt_req_read_multi_handler
 *
 * Process write read multi request from peer device
 *
 * Params:	conn_id       			Connection ID
 * 			opcode        			BLE GATT request type opcode
 * 			p_read_req    			Pointer to read request containing the handle to read
 * 			len_requested 			Length of data requested
 * Return:	wiced_bt_gatt_status_t  BLE GATT status
*******************************************************************************/
static wiced_bt_gatt_status_t app_bt_gatt_req_read_multi_handler(uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
                                                  wiced_bt_gatt_read_multiple_req_t *p_read_req, uint16_t len_requested)
{
    gatt_db_lookup_table_t *puAttribute;
    uint8_t *p_rsp = app_bt_alloc_buffer(len_requested);
    int used = 0;
    int xx;
    uint16_t handle = wiced_bt_gatt_get_handle_from_stream(p_read_req->p_handle_stream, 0);

    if (p_rsp == NULL)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, handle, WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Read by type returns all attributes of the specified type, between the start and end handles */
    for (xx = 0; xx < p_read_req->num_handles; xx++)
    {
        handle = wiced_bt_gatt_get_handle_from_stream(p_read_req->p_handle_stream, xx);
        if ((puAttribute = app_bt_find_by_handle(handle)) == NULL)
        {
            wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, *p_read_req->p_handle_stream, WICED_BT_GATT_ERR_UNLIKELY);
            app_bt_free_buffer(p_rsp);
            return WICED_BT_GATT_ERR_UNLIKELY;
        }

        {
            int filled = wiced_bt_gatt_put_read_multi_rsp_in_stream(opcode, p_rsp + used, len_requested - used,
                                                        puAttribute->handle, puAttribute->cur_len, puAttribute->p_data);
            if (!filled)
            {
                break;
            }
            used += filled;
        }
    }

    if (used == 0)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, *p_read_req->p_handle_stream, WICED_BT_GATT_INVALID_HANDLE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

	switch ( *p_read_req->p_handle_stream )
	{
		// Add action when specified handle is read
		case HDLC_PSOC_LED_VALUE:
			printf( "LED is %s\r\n", app_psoc_led[0] ? "ON" : "OFF" );
			break;
	}

    /* Send the response */
    wiced_bt_gatt_server_send_read_multiple_rsp(conn_id, opcode, used, p_rsp, (void *)app_bt_free_buffer);

    return WICED_BT_GATT_SUCCESS;
}


/*******************************************************************************
* Function Name: app_bt_find_by_handle
*
* Finds attribute location by handle
*
* Param:  handle    				handle to look up
* Return: gatt_db_lookup_table_t   	pointer to location containing handle data
********************************************************************************/
static gatt_db_lookup_table_t *app_bt_find_by_handle(uint16_t handle)
{
    int i;
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == handle)
        {
            return (&app_gatt_db_ext_attr_tbl[i]);
        }
    }
    return NULL;
}


/*******************************************************************************
* Function Name: app_bt_alloc_buffer
*
* This Function allocates the buffer of requested length
*
* Param:  len			Length of buffer
* Return: uint8_t*      Pointer to allocated buffer
********************************************************************************/
static uint8_t *app_bt_alloc_buffer(uint16_t len)
{
    uint8_t *p = (uint8_t *)malloc(len);
    return p;
}

/*******************************************************************************
* Function Name: app_bt_free_buffer
*
* This Function frees the buffer requested
*
* Param:  p_data		Pointer to buffer to be freed
********************************************************************************/
static void app_bt_free_buffer(uint8_t *p_data)
{
    if (p_data != NULL)
    {
        free(p_data);
    }
}
/* [] END OF FILE */
