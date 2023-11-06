#include "cybsp.h"
#include "cyhal.h"
#include "cy_retarget_io.h"

/* Library for malloc and free */
#include "stdlib.h"

/* FreeRTOS */
#include <FreeRTOS.h>
#include <task.h>

/* btstack */
#include "wiced_bt_stack.h"

/* App utilities */
#include "app_bt_utils.h"

/* Header files from BT configurator */
#include "cycfg_bt_settings.h"
#include "cycfg_gap.h"
#include "cycfg_gatt_db.h"
#include "BMP180.h"

/*******************************************************************
 * Macros to assist development of the exercises
 ******************************************************************/

#define TASK_STACK_SIZE (1024u)
#define	TASK_PRIORITY 	(5u)

#define GPIO_INTERRUPT_PRIORITY 7U
#define PORT_INTR_MASK  (0x00000001UL << 0U) //Address port

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

uint8_t button_notify = 0;
uint8_t rx_buf[32];

uint16_t dataRead;
uint16_t connection_id = 0;

uint32_t rx_len = 32;

float press,tp;

TaskHandle_t NotifyTaskHandle = NULL;
TaskHandle_t NotifyTaskHandle1 = NULL;
TaskHandle_t NotifyTaskHandle2 = NULL;

cy_stc_scb_uart_context_t UART_context;
cy_stc_scb_i2c_context_t i2cContext;

extern bmp180_obj bmp180;

void scaning_i2c()
{
	for(uint8_t i=1; i<0x7f; i++)
	{
		if( Cy_SCB_I2C_MasterSendStart(i2c_HW, i, CY_SCB_I2C_WRITE_XFER, 10UL, &i2cContext) == CY_SCB_I2C_SUCCESS )
		printf(" found device on address: 0x%x \r\n", i);
		Cy_SCB_I2C_MasterSendStop(i2c_HW, 10UL, &i2cContext);
	}
}

uint32_t bmp_write_bytes(uint8_t addr, uint8_t *data, uint8_t dataLen, uint32_t timeout)
{
	cy_en_scb_i2c_status_t status=0;
	if( Cy_SCB_I2C_MasterSendStart(i2c_HW, (uint32_t)addr, CY_SCB_I2C_WRITE_XFER, timeout, &i2cContext) == CY_SCB_I2C_SUCCESS )
		{
			for(uint8_t i = 0; i<dataLen; i++)
			status |= Cy_SCB_I2C_MasterWriteByte (i2c_HW, data[i], timeout,&i2cContext);
		}
	status = Cy_SCB_I2C_MasterSendStop(i2c_HW, 10UL, &i2cContext);
	return (uint32_t)status;
}

uint32_t bmp_read_bytes(uint8_t addr, uint8_t reg, uint8_t *val, uint8_t dataLen, uint32_t timeout)
{
	cy_en_scb_i2c_status_t status=0;
	if( Cy_SCB_I2C_MasterSendStart(i2c_HW, (uint32_t)addr, CY_SCB_I2C_WRITE_XFER, timeout, &i2cContext) == CY_SCB_I2C_SUCCESS )
		{
		if( CY_SCB_I2C_SUCCESS == Cy_SCB_I2C_MasterWriteByte (i2c_HW, reg, timeout,&i2cContext) )
			{
				Cy_SCB_I2C_MasterSendReStart(i2c_HW, (uint32_t)addr, CY_SCB_I2C_READ_XFER, timeout, &i2cContext);
				for( uint8_t i = 0; i< dataLen-1; i++)
					Cy_SCB_I2C_MasterReadByte (i2c_HW, CY_SCB_I2C_ACK, val++, timeout, &i2cContext);
					Cy_SCB_I2C_MasterReadByte (i2c_HW, CY_SCB_I2C_NAK, val, timeout, &i2cContext);
			}
		}
	status = Cy_SCB_I2C_MasterSendStop(i2c_HW, 10UL, &i2cContext);
	return (uint32_t)status;
}

void Interrupt_Handler(void){
	// Get interrupt cause
	uint32_t intrSrc = Cy_GPIO_GetInterruptCause0();
	/* Check if the interrupt was from the button's port */
	if(PORT_INTR_MASK == (intrSrc & PORT_INTR_MASK)){
		/* Clear the interrupt */
		Cy_GPIO_ClearInterrupt(P0_4_PORT, P0_4_NUM);

		BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		/* Increment button counter */
		if( app_psoc_button_count_client_char_config[0] & GATT_CLIENT_CONFIG_NOTIFICATION )
		app_psoc_button_count[0]++;

		else if( app_psoc_button_count_2_client_char_config[0] & GATT_CLIENT_CONFIG_NOTIFICATION )
		app_psoc_button_count[0]--;

		/* Notify the counter task that the button was pressed */
		vTaskNotifyGiveFromISR( NotifyTaskHandle, &xHigherPriorityTaskWoken );

		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}

static void BMP180_task(void * arg){
	char buffer_text[40];
	while(true)
	{
		bmp180.readValue(&tp, &press, 100);
		printf("Temp : %0.2f \tPress : %0.2f \r\n", tp, press);

		if(app_psoc_bmp180_client_char_config[0] & GATT_CLIENT_CONFIG_NOTIFICATION){
			sprintf(buffer_text,"Temp : %0.2f \t Press : %0.2f",tp,press);
			wiced_bt_gatt_server_send_notification( connection_id,
													HDLC_PSOC_BMP180_VALUE,
													strlen(buffer_text),
													(uint8_t *)buffer_text,
													NULL);
		}
		vTaskDelay(250);
	}
}

static void US100_task(void * arg){
	char buffer_text[24];
	while(true)
	{
    		uint8_t cmd = 0X55;
    				Cy_SCB_UART_PutArrayBlocking(UART_HW, &cmd, 1);
	        		Cy_SCB_UART_GetArrayBlocking(UART_HW, rx_buf, 2);
	        		dataRead = (uint16_t)rx_buf[0] << 8 | rx_buf[1];
	                printf("US100 = %d\r\n\n",dataRead);

		if (connection_id)
		{
			if( app_psoc_us100_client_char_config[0] & GATT_CLIENT_CONFIG_NOTIFICATION )
			{
				sprintf(buffer_text,"US100 = %d", dataRead);
				wiced_bt_gatt_server_send_notification( connection_id,
														HDLC_PSOC_US100_VALUE,
														strlen(buffer_text),
														(uint8_t *)buffer_text,
														NULL);
			}
		}
		vTaskDelay(250);
	}
}

static void notify_task(void * arg)
{
	/* Notification values received from ISR */
	uint32_t ulNotificationValue;
	char buffer_text[24];
	while(true)
	{
		/* Wait for the button ISR */
		ulNotificationValue = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
		if (ulNotificationValue == 1)
		{
			if( connection_id ) /* Check if we have an active connection */
			{
				/* Check to see if the client has asked for notifications */
				if( app_psoc_button_count_client_char_config[0] & GATT_CLIENT_CONFIG_NOTIFICATION )
				{
					printf( "Notify button press count: (%d)\n",
					app_psoc_button_count[0] );
					sprintf(buffer_text,"Button Count = %d",app_psoc_button_count[0]);
					wiced_bt_gatt_server_send_notification( connection_id,
															HDLC_PSOC_BUTTON_COUNT_VALUE,
															strlen(buffer_text),
															(uint8_t *)buffer_text,
															NULL);
				}
				if( app_psoc_button_count_2_client_char_config[0] & GATT_CLIENT_CONFIG_NOTIFICATION )
				{
					printf( "Notify button press count: (%d)\n",
					app_psoc_button_count[0] );
					sprintf(buffer_text,"Button Count = %d",app_psoc_button_count[0]);
					wiced_bt_gatt_server_send_notification( connection_id,
															HDLC_PSOC_BUTTON_COUNT_2_VALUE,
															strlen(buffer_text),
															(uint8_t *)buffer_text,
															NULL);
				}
			}
		}
	vTaskDelay(250);
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
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    // Initialize pin P0.4
    result = Cy_GPIO_Pin_Init(P0_4_PORT, P0_4_NUM, &CYBSP_SW2_config);
    if(result != CY_RSLT_SUCCESS)
    {
 	   CY_ASSERT(0);
    }

    result = Cy_GPIO_Pin_Init(P7_1_PORT, P7_1_NUM, &CYBSP_LED4_config);
    if(result != CY_RSLT_SUCCESS)
    {
    	   CY_ASSERT(0);
    }

    cy_stc_sysint_t intrCfg =
    {
    .intrSrc = ioss_interrupts_gpio_0_IRQn,
    .intrPriority = GPIO_INTERRUPT_PRIORITY
    };

    Cy_SysInt_Init(&intrCfg, &Interrupt_Handler);
    NVIC_EnableIRQ(intrCfg.intrSrc);

    printf("**********Application Start*****************\n");

    /* Configure platform specific settings for the BT device */
    cybt_platform_config_init(&cybsp_bt_platform_cfg);

    /* Initialize stack and register the callback function */
    wiced_bt_stack_init (app_bt_management_callback, &wiced_bt_cfg_settings);

    // Initialize the TCPWM block
	Cy_TCPWM_PWM_Init(pwm_HW, pwm_NUM, &pwm_config);
	// Enable the TCPWM block
	Cy_TCPWM_PWM_Enable(pwm_HW, pwm_NUM);
	// Start the PWM
	Cy_TCPWM_TriggerReloadOrIndex(pwm_HW, pwm_MASK);

     result = Cy_SCB_UART_Init(UART_HW, &UART_config, &UART_context);
     if(result == CY_RSLT_SUCCESS)printf("Success initialization UART US100\r\n\n");
     Cy_SCB_UART_Enable(UART_HW);

 	if( Cy_SCB_I2C_Init(i2c_HW, &i2c_config, &i2cContext) == CY_SCB_I2C_SUCCESS )
 		{
 			printf("I2C success \r\n");
 			Cy_SCB_I2C_Enable(i2c_HW);
 		}

 	scaning_i2c();

 	if( bmp180.init(BMP180_MODE_STANDARD, BMP180_ADDR, bmp_read_bytes, bmp_write_bytes) == BMP180_SUCCESS )
 		printf("Device Found !\r\n");

    xTaskCreate (notify_task, "NotifyTask", TASK_STACK_SIZE, NULL, TASK_PRIORITY, &NotifyTaskHandle);
    xTaskCreate (US100_task, "US100Task", TASK_STACK_SIZE, NULL, TASK_PRIORITY-1, &NotifyTaskHandle1);
    xTaskCreate (BMP180_task, "BMP180Task", TASK_STACK_SIZE, NULL, TASK_PRIORITY-2, &NotifyTaskHandle2);

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

				/* Set the local BDA from the value in the configurator */
				wiced_bt_set_local_bdaddr((uint8_t *)cy_bt_device_address, BLE_ADDR_PUBLIC);
				wiced_bt_dev_read_local_addr( bda );
				printf( "Local Bluetooth® Device Address: ");
				print_bd_address(bda);

				/*Register GATT callback and initialize database*/
				wiced_bt_gatt_register( app_bt_gatt_event_callback );
				wiced_bt_gatt_db_init( gatt_database, gatt_database_len, NULL );

				/*Disable pairing */
				wiced_bt_set_pairable_mode( WICED_TRUE, WICED_FALSE );

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
            if (p_event_data->ble_advert_state_changed == BTM_BLE_ADVERT_OFF){
            	if(connection_id == 0){
            		Cy_TCPWM_PWM_SetCompare0(pwm_HW, pwm_NUM, 0);
            	}
            	else { //connected
            		Cy_TCPWM_PWM_SetCompare0(pwm_HW, pwm_NUM, 500);
            	}
            }
            else{
            	Cy_TCPWM_PWM_SetCompare0(pwm_HW, pwm_NUM, 25);
            }
            result = WICED_BT_SUCCESS;
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
    wiced_bt_gatt_write_req_t write_param = p_data->attribute_request.data.write_req;
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

			printf("BLE WRITE Request\n\r");
			printf("Value Id = %d\n\r",write_param.handle); //Untuk mengetahui id dari karakteristik notify
			printf("%d\r\n",write_param.p_val[0]);

			if(write_param.handle == HDLD_PSOC_BUTTON_COUNT_CLIENT_CHAR_CONFIG)
				button_notify = write_param.p_val[0];

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
                // TODO Add action when specified handle is written
                 case HDLC_PSOC_LED_VALUE:
                 Cy_GPIO_Write(P7_1_PORT, P7_1_NUM, app_psoc_led[0] == 0);
                 printf( "Turn the LED %\r\n", app_psoc_led[0] ? "ON" : "OFF" );
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
        // TODO Add action when specified handle is read
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
        // TODO Add action when specified handle is read
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
        // TODO Add action when specified handle is read
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
