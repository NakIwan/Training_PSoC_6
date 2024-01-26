/* Output = 1 (singgle click) digunakan untuk Up Command,
 * Output = 2 (double click) digunakan untuk Down Command,
 * Output = 3 (triple click) digunakan untuk Back Command,
 * Output = 100 (long press) digunakan untuk enter Command
 *
 * Screen 1 = Main screen,
 * Screen 2 = setting jam dan tanggal,
 * Screen 3 = Menampilkan data sensor BMP280 atau DPS310
 * Screen 4 = Menampilkan data sensor PAS CO2
 * Menu yang tersedia = setting jam (1), environment(2), and air quality(3)
 */

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

/*Include FreeRTOS*/
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "queue.h"
#include "task.h"

#include "GUI.h"
#include "mtb_ssd1306.h"
#include "interface.h"
//#include "button.h"

#define taskPriority  6

#define MAIN_SCREEN 		1
#define SET_CLOCK_SCREEN 	2
#define BMP280_SCREEN 		3
#define PASCO2_SCREEN 		4

#define SET_CLOCK	1
#define BMP280		2
#define PASCO2		3

TaskHandle_t buttonHandle;
TaskHandle_t displayHandle;

QueueHandle_t menu_;
QueueHandle_t screen_;

interface_t interface_obj;
cyhal_i2c_t i2c;

bool upCmd = false;
bool downCmd = false;
bool backCmd = false;
bool enterCmd = false;

const cyhal_i2c_cfg_t i2c_cfg =
{
		.is_slave 		 = false,
		.address  		 = 0,
		.frequencyhal_hz = 400000
};

//typedef struct {
//	const char dispTitle[22];
//}title_t;
//
//title_t title [] = {
//		{"Main Display"},
//		{"Seting Date & Time"}
//};

const char *menu[] = {"Time & Date", "Environmental Data", "Air Quality"};

const char testDisp [] = {"Test Display"};

void displayOled(void *arg){

	cyhal_i2c_init(&i2c, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);
	cyhal_i2c_configure(&i2c, &i2c_cfg);

	interface_construct(&interface_obj, &i2c);
	interface_begin(&interface_obj);
	interface_setTitle(&interface_obj, testDisp);
	interface_set_menu(&interface_obj, menu, 3);


	uint8_t menu, screen;
	while(1){

		interface_draw_menu(&interface_obj);
		vTaskDelay(20);

	}
}

void ButtonApp(void *arg){

	uint8_t State, lastState;
	uint8_t lastButtonState;
	uint8_t	output;

	uint8_t current_screen = 1;
	uint8_t menu_item = 1;

	unsigned long holdTime = 500, DBInterval = 50, RO_Time;
	unsigned long time, duration = 1000, HeldTime;
	unsigned long lastDebounceTime;

	for(;;){

		output = 0;
		uint8_t btnApp = cyhal_gpio_read(CYBSP_USER_BTN);
		if ( btnApp != lastButtonState){
			lastDebounceTime = xTaskGetTickCount();
			lastButtonState = btnApp;
			time = xTaskGetTickCount();
		}

		RO_Time = xTaskGetTickCount();
		if (RO_Time > xTaskGetTickCount())
			time = RO_Time;

		while ((xTaskGetTickCount() - time) <= duration){
			if (btnApp != lastState){
				if (btnApp == !State){
					if ((xTaskGetTickCount() - lastDebounceTime) >= DBInterval){
						output++;
						lastDebounceTime = xTaskGetTickCount();
					}
				}
				lastState = cyhal_gpio_read(CYBSP_USER_BTN);
			}
			btnApp = cyhal_gpio_read(CYBSP_USER_BTN);
		}

		if (btnApp == State && btnApp == lastButtonState)
			if ((HeldTime = (xTaskGetTickCount() - time)) > holdTime)
				output = 100;

		/* Up Command*/
		if (output == 1 && upCmd == false){
			upCmd = true;
			if (upCmd && output == 1)
			{
				upCmd = false;
				menu_item --;
				interface_provious(&interface_obj);
				if (menu_item < SET_CLOCK)
					menu_item = PASCO2;
			}
		}

		/* Down command*/
		if (output == 2 && downCmd == false){
			downCmd = true;
			if (downCmd == true && output == 2)
			{
				downCmd = false;
				menu_item ++;
				interface_next(&interface_obj);
				if (menu_item > PASCO2)
					menu_item = SET_CLOCK;
			}
		}

		/* Back Command */
		if (output == 3 && backCmd == false)
		{
			backCmd = true;
			if (backCmd == true && output == 3){
				backCmd = false;
				if (current_screen == SET_CLOCK_SCREEN || current_screen == BMP280_SCREEN || current_screen == PASCO2_SCREEN)
					{
						current_screen = MAIN_SCREEN;
					}
			}
		}

		/* Enter Command*/
		if (output == 100 && enterCmd == false){
			enterCmd = true;
			if (enterCmd == true && output == 100)
			{
				enterCmd = false;
				interface_reset_position(&interface_obj);
				if (current_screen == MAIN_SCREEN && menu_item == SET_CLOCK)
					current_screen = SET_CLOCK_SCREEN;

				else if (current_screen == MAIN_SCREEN && menu_item == BMP280)
					current_screen = BMP280_SCREEN;

				else if (current_screen == MAIN_SCREEN && menu_item == PASCO2)
					current_screen = PASCO2_SCREEN;
			}
		}
		printf("Poss %d\r\n",interface_getPosition(&interface_obj));
//		xQueueSend(menu_, &menu_item, pdMS_TO_TICKS(10));
//		xQueueSend(screen_, &current_screen, pdMS_TO_TICKS(10));
		vTaskDelay(5);
	}
}

int main(void)
{
	cy_rslt_t result;

	menu_ = xQueueCreate(1,sizeof(uint8_t));
	screen_ = xQueueCreate(1,sizeof(uint8_t));

#if defined (CY_DEVICE_SECURE)
	cyhal_wdt_t wdt_obj;

	/* Clear watchdog timer so that it doesn't trigger a reset */
	result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
	CY_ASSERT(CY_RSLT_SUCCESS == result);
	cyhal_wdt_free(&wdt_obj);
#endif

	/* Initialize the device and board peripherals */
	result = cybsp_init();
	if (result != CY_RSLT_SUCCESS)
	{
		CY_ASSERT(0);
	}

	/* Enable global interrupts */
	__enable_irq();

	cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
	cyhal_gpio_init(CYBSP_USER_BTN,CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, 1ul);

	xTaskCreate(ButtonApp, "ButtonApp", 1024, NULL, taskPriority, &buttonHandle);
	xTaskCreate(displayOled, "DisplayApp", 1024, NULL, (taskPriority-1), &displayHandle);

	vTaskStartScheduler();

	CY_ASSERT(0);

}


//void displayOled(void *arg){
//
//	cyhal_i2c_init(&i2c, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);
//	cyhal_i2c_configure(&i2c, &i2c_cfg);

//	interface_construct(&interface_obj, &i2c);
//	interface_begin(&interface_obj);
//	interface_clear(&interface_obj);


//	mtb_ssd1306_init_i2c(&i2c);
//	GUI_Init();
//	GUI_Clear();

//	uint8_t menu, screen;
//	while(1){

//		interface_setTitle(&interface_obj, testDisp);
//		interface_setMenu(&interface_obj, testDisp, 4);

//		xQueueReceive(menu_, &menu, pdMS_TO_TICKS(10));
//		xQueueReceive(screen_, &screen, pdMS_TO_TICKS(10));
//		GUI_Clear();
//		if (screen == 1){
//			GUI_DispStringAt("Smart Clock", 30, 5);
//
//			if (menu == 1){
//				GUI_DispStringAt("> Setting Clock", 5, 30);
//			}
//			else
//				GUI_DispStringAt("  Setting Clock", 5, 30);
//
//			if (menu == 2){
//				GUI_DispStringAt("> Environment", 5, 40);
//			}
//			else {
//				GUI_DispStringAt("  Environment", 5, 40);
//			}
//
//			if (menu == 3){
//				GUI_DispStringAt("> Air Quality", 5, 50);
//			}
//			else{
//				GUI_DispStringAt("  Air Quality", 5, 50);
//			}
//		}
//		else if (screen == 2){
//			GUI_DispStringAt("Setting Time & Date", 5,5);
//			GUI_DispStringAt("HH: ,MM: ,SS: \r\n\nDD: ,MM: ,YY: ", 0, 25);
//		}
//		if (screen == 3){
//			GUI_DispStringAt("BMP280 Sensor", 15,5);
//			GUI_DispStringAt("Press : 1020 hPa", 5, 25);
//			GUI_DispStringAt("Temp  : 26 C", 5, 35);
//		}
//		else if (screen == 4){
//			GUI_DispStringAt("PAS CO2 Sensor", 15,5);
//			GUI_DispStringAt("Co2 : 800 ppm", 5, 25);
//		}
//
//		printf("Screen: %d\r\n", screen);
//		printf("Menu  : %d\r\n", menu);
//		vTaskDelay(20);
//
//	}
//}
