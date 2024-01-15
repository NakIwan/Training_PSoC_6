#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "stdlib.h"
#include "stdio.h"
#include "BMP280.h"

#include "cyberon_asr.h"
#include "wave.h"

#ifdef USE_WM8960
	#include "mtb_wm8960.h"
#endif

#define FRAME_SIZE                  (960u)
#define SAMPLE_RATE_HZ              (16000u)
#define DECIMATION_RATE             (96u)
//#define AUDIO_SYS_CLOCK_HZ          (24576000u)
#define AUDIO_SYS_CLOCK_HZ          (98000000u)
#define HFCLK1_CLK_DIVIDER  		(4u)
#define DEBOUNCE_DELAY_MS   		(10u)
#define PDM_DATA                    (P10_5)
#define PDM_CLK                     (P10_4)
#define LED_PIN						(P13_7)
#define FREQ						(850u)

void pdm_pcm_isr_handler(void *arg, cyhal_pdm_pcm_event_t event);
void clock_init(void);
void i2s_isr_handler(void* arg, cyhal_i2s_event_t event);

void asr_callback(const char *function, char *message, char *parameter);

/*Variable Global*/
#ifdef USE_WM8960
	cyhal_i2c_t i2c;
#endif
	cyhal_i2s_t i2s;
	cyhal_clock_t   audio_clock;
	cyhal_clock_t   pll_clock;
	cyhal_clock_t   fll_clock;
	cyhal_clock_t   system_clock;
	cyhal_pwm_t pwm_obj;

volatile bool pdm_pcm_flag = false;
bool flag_play = false;
int16_t pdm_pcm_ping[FRAME_SIZE] = {0};
int16_t pdm_pcm_pong[FRAME_SIZE] = {0};
int16_t *pdm_pcm_buffer = &pdm_pcm_ping[0];
int16_t *pdm_pcm_buffer1 = &pdm_pcm_pong[0];
cyhal_pdm_pcm_t pdm_pcm;

float duty_cycle = 100.f;

#ifdef USE_WM8960
	const cyhal_i2c_cfg_t mi2c_cfg =
	{
		.is_slave 		 = false,
		.address  		 = 0,
		.frequencyhal_hz = 400000
	};
#endif

const cyhal_i2s_pins_t i2s_pins = {
	    .sck  = P9_1,
	    .ws   = P9_2,
	    .data = P9_3,
	    .mclk = NC,
};

const cyhal_i2s_config_t i2s_config = {
    .is_tx_slave    = false,    /* TX is Master */
    .is_rx_slave    = false,    /* RX not used */
    .mclk_hz        = 0,        /* External MCLK not used */
    .channel_length = 32,       /* In bits */
    .word_length    = 16,       /* In bits */
    .sample_rate_hz = 24000,    /* In Hz */

};
const cyhal_pdm_pcm_cfg_t pdm_pcm_cfg = 
{
    .sample_rate     = SAMPLE_RATE_HZ,
    .decimation_rate = DECIMATION_RATE,
    .mode            = CYHAL_PDM_PCM_MODE_RIGHT,
    .word_length     = 16,  /* bits */
    .left_gain       = CYHAL_PDM_PCM_MAX_GAIN,   /* dB */
    .right_gain      = CYHAL_PDM_PCM_MAX_GAIN,   /* dB */
};

int main(void)
{
    cy_rslt_t result;
    uint64_t uid;

    result = cybsp_init() ;
    if(result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();
    clock_init();
    cyhal_system_delay_ms(1);
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    printf("\x1b[2J\x1b[;H");
    printf("\r\n");
    printf("===== Cyberon DSpotter Demo =====\r\n");

    cyhal_pwm_init(&pwm_obj, LED_PIN, NULL);
    cyhal_pwm_set_duty_cycle(&pwm_obj, duty_cycle, FREQ);
    cyhal_pwm_start(&pwm_obj);

    cyhal_i2s_init(&i2s, &i2s_pins, NULL, &i2s_config, &audio_clock);
    cyhal_i2s_register_callback(&i2s, i2s_isr_handler, NULL);
    cyhal_i2s_enable_event(&i2s, CYHAL_I2S_ASYNC_TX_COMPLETE, CYHAL_ISR_PRIORITY_DEFAULT, true);
    printf("\r\n");

#ifdef USE_WM8960
    cyhal_i2c_init(&i2c, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);
    cyhal_i2c_configure(&i2c, &mi2c_cfg);

    mtb_wm8960_init(&i2c, WM8960_FEATURE_HEADPHONE);
    mtb_wm8960_activate();
    mtb_wm8960_adjust_heaphone_output_volume(0X7f);
#endif

    cyhal_pdm_pcm_init(&pdm_pcm, PDM_DATA, PDM_CLK, &audio_clock, &pdm_pcm_cfg);
    cyhal_pdm_pcm_register_callback(&pdm_pcm, pdm_pcm_isr_handler, NULL);
    cyhal_pdm_pcm_enable_event(&pdm_pcm, CYHAL_PDM_PCM_ASYNC_COMPLETE, CYHAL_ISR_PRIORITY_DEFAULT, true);
    cyhal_pdm_pcm_start(&pdm_pcm);
    cyhal_pdm_pcm_read_async(&pdm_pcm, pdm_pcm_buffer, FRAME_SIZE);

    uid = Cy_SysLib_GetUniqueId();
    printf("uniqueIdHi: 0x%08lX, uniqueIdLo: 0x%08lX\r\n", (uint32_t)(uid >> 32), (uint32_t)(uid << 32 >> 32));
    printf("\r\n");
    if(!cyberon_asr_init(asr_callback))
    {
    	while(1);
    }

//    if (bmp280.init(&i2c, BMP280_ADDR) == BMP280_SUCCESS)
    	printf("Waiting Say Sky\r\n");

    float last_Dc = duty_cycle;
//    float press, temp;

    while(1)
    {
//    	BMP280_readValue(&temp, &press, 100);
//    	printf("Temp : %0.2f\tPress : %0.2f\r\n",temp,press);

    	if(last_Dc != duty_cycle){
    		cyhal_pwm_set_duty_cycle(&pwm_obj, duty_cycle, FREQ);
			cyhal_pwm_start(&pwm_obj);
			last_Dc = duty_cycle;
    	}

    	if(flag_play){
    		if(cyhal_i2s_is_write_pending(&i2s)){

    		}
    		else {
    			cyhal_i2s_start_tx(&i2s);
    			cyhal_i2s_write_async(&i2s, wave_data, WAVE_SIZE);
    			printf("Play Sound feedback");
    		}
    	}

        if(pdm_pcm_flag)
        {
            pdm_pcm_flag = 0;
            cyberon_asr_process(pdm_pcm_buffer, FRAME_SIZE);
        }
    }
}

void pdm_pcm_isr_handler(void *arg, cyhal_pdm_pcm_event_t event)
{
    static bool ping_pong = false;

    (void) arg;
    (void) event;

    if(ping_pong)
    {
        cyhal_pdm_pcm_read_async(&pdm_pcm, pdm_pcm_ping, FRAME_SIZE);
        pdm_pcm_buffer = &pdm_pcm_pong[0];
    }
    else
    {
        cyhal_pdm_pcm_read_async(&pdm_pcm, pdm_pcm_pong, FRAME_SIZE);
        pdm_pcm_buffer = &pdm_pcm_ping[0]; 
    }

    ping_pong = !ping_pong;
    pdm_pcm_flag = true;
}

void clock_init(void)
{
//	cyhal_clock_reserve(&pll_clock, &CYHAL_CLOCK_PLL[0]);
//    cyhal_clock_set_frequency(&pll_clock, AUDIO_SYS_CLOCK_HZ, NULL);
//    cyhal_clock_set_enabled(&pll_clock, true, true);
//
//    cyhal_clock_reserve(&audio_clock, &CYHAL_CLOCK_HF[1]);
//
//    cyhal_clock_set_source(&audio_clock, &pll_clock);
//    cyhal_clock_set_enabled(&audio_clock, true, true);

    /* Initialize the PLL */
    cyhal_clock_reserve(&pll_clock, &CYHAL_CLOCK_PLL[1]);
    cyhal_clock_set_frequency(&pll_clock, AUDIO_SYS_CLOCK_HZ, NULL);
    cyhal_clock_set_enabled(&pll_clock, true, true);

    /* Initialize the audio subsystem clock (HFCLK1) */
    cyhal_clock_reserve(&audio_clock, &CYHAL_CLOCK_HF[1]);
    cyhal_clock_set_source(&audio_clock, &pll_clock);

    /* Drop HFCK1 frequency for power savings */
    cyhal_clock_set_divider(&audio_clock, HFCLK1_CLK_DIVIDER);
    cyhal_clock_set_enabled(&audio_clock, true, true);

    /* Initialize the system clock (HFCLK0) */
    cyhal_clock_reserve(&system_clock, &CYHAL_CLOCK_HF[0]);
    cyhal_clock_set_source(&system_clock, &pll_clock);

    /* Disable the FLL for power savings */
    cyhal_clock_reserve(&fll_clock, &CYHAL_CLOCK_FLL);
    cyhal_clock_set_enabled(&fll_clock, false, true);
}

void asr_callback(const char *function, char *message, char *parameter)
{
	printf("[%s]%s(%s)\r\n", function, message, parameter);
	uint32_t cmd_id = atoi(parameter);
	command(cmd_id);
}

void command(uint32_t cmd_id){
	switch (cmd_id){
	case 100:
		duty_cycle = 0.f;
		flag_play = true;
		break;
	case 101:
		duty_cycle = 100.f;
		break;
	case 102:
		duty_cycle = 50.f;
		break;
	default:
		break;
	};
}

void i2s_isr_handler(void *arg, cyhal_i2s_event_t event)
{
    (void) arg;
    (void) event;

    /* Stop the I2S TX */
    cyhal_i2s_stop_tx(&i2s);
    flag_play = false;
}
