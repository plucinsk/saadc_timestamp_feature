/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
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
 * 
 */
 
#include <stdint.h>
#include <string.h>
#

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "nrf.h"
#include "boards.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_delay.h"

#include "nrf_drv_power.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "arm_const_structs.h"

// SAADC DEFINES
#define SAADC_CALIBRATION_INTERVAL      5
#define SAADC_SAMPLES_IN_BUFFER         250
#define SAADC_OVERSAMPLE                NRF_SAADC_OVERSAMPLE_DISABLED
#define SAADC_BURST_MODE                0

// RTC DEFINES
#define RTC_FREQUENCY 1000
#define RTC_CC_VALUE 4

// PPI DEFINES
#define PPI_TIMER1_INTERVAL   (1000)


#define UART_PRINTING_ENABLED

#ifdef UART_PRINTING_ENABLED
static uint32_t m_adc_evt_counter = 0;
#endif

const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2);
static nrf_saadc_value_t m_buffer_pool[2][SAADC_SAMPLES_IN_BUFFER];
static bool m_saadc_initialized = false;

static const nrf_drv_timer_t m_counter_timer = NRF_DRV_TIMER_INSTANCE(0);
static const nrf_drv_timer_t m_event_timer   = NRF_DRV_TIMER_INSTANCE(1);

static nrf_ppi_channel_t m_ppi_channel1;

static void counter_timer_event_handler(nrf_timer_event_t event_type, void * p_context)
{

}

static void event_timer_event_handler(nrf_timer_event_t event_type, void * p_context)
{
    uint32_t count = nrf_drv_timer_capture(&m_counter_timer, NRF_TIMER_CC_CHANNEL0);
}

nrf_log_timestamp_func_t timestamp(void)
{
    return m_counter_timer.p_reg->CC[0];
}

static void ppi_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel1);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel1,
                                          nrf_drv_timer_event_address_get(&m_event_timer,
                                                                          NRF_TIMER_EVENT_COMPARE0),
                                          nrf_drv_timer_task_address_get(&m_counter_timer,
                                                                         NRF_TIMER_TASK_COUNT));
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_enable(m_ppi_channel1);
    APP_ERROR_CHECK(err_code);
}

static void counter_timer_init(void)
{
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.frequency = NRF_TIMER_FREQ_31250Hz;
    timer_cfg.mode = NRF_TIMER_MODE_COUNTER;

    ret_code_t err_code = nrf_drv_timer_init(&m_counter_timer, &timer_cfg, counter_timer_event_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_enable(&m_counter_timer);
}

static void event_timer_init(void)
{
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.frequency = NRF_TIMER_FREQ_31250Hz;

    ret_code_t err_code = nrf_drv_timer_init(&m_event_timer, &timer_cfg, event_timer_event_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_extended_compare(&m_event_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   nrf_drv_timer_ms_to_ticks(&m_event_timer, PPI_TIMER1_INTERVAL),
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   true);
    nrf_drv_timer_enable(&m_event_timer);
}


static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    ret_code_t err_code;

    if (int_type == NRF_DRV_RTC_INT_COMPARE0)
    {
        if(!m_saadc_initialized)
        {
            saadc_init();                                              //Initialize the SAADC. In the case when SAADC_SAMPLES_IN_BUFFER > 1 then we only need to initialize the SAADC when the the buffer is empty.
        }
        m_saadc_initialized = true;                                    //Set SAADC as initialized

        nrf_drv_saadc_sample();                                        //Trigger the SAADC SAMPLE task
			
        LEDS_INVERT(BSP_LED_2_MASK);                                   //Toggle LED1 to indicate SAADC sampling start
			
        err_code = nrf_drv_rtc_cc_set(&rtc,0,RTC_CC_VALUE,true);       //Set RTC compare value. This needs to be done every time as the nrf_drv_rtc clears the compare register on every compare match
        APP_ERROR_CHECK(err_code);
        nrf_drv_rtc_counter_clear(&rtc);                               //Clear the RTC counter to start count from zero
    }
}

static void lfclk_config(void)
{
    ret_code_t err_code;
    
    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}

static void rtc_config(void)
{
    ret_code_t err_code;
    nrf_drv_rtc_config_t rtc_config;

    rtc_config.prescaler = RTC_FREQ_TO_PRESCALER(RTC_FREQUENCY);
    err_code = nrf_drv_rtc_init(&rtc, &rtc_config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_rtc_cc_set(&rtc, 0, RTC_CC_VALUE, true);
    APP_ERROR_CHECK(err_code);

    nrf_drv_rtc_enable(&rtc);
}

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    float32_t m_rms_input_ain0[SAADC_SAMPLES_IN_BUFFER];
    //float32_t m_rms_input_ain1[SAADC_SAMPLES_IN_BUFFER / 2];
    float32_t m_rms_output_ain0;
    //float32_t m_rms_output_ain1;
    uint32_t rms_value;
    uint32_t prev_rms_value = 0;
    if (p_event -> type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;
        LEDS_INVERT(BSP_LED_3_MASK);

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        if(m_adc_evt_counter % SAADC_CALIBRATION_INTERVAL == 0)
        {
            #ifdef UART_PRINTING_ENABLED
                NRF_LOG_INFO("SAADC Calibration started ...\r\n");
            #endif
            NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
            nrf_saadc_task_trigger(NRF_SAADC_TASK_CALIBRATEOFFSET);
            while(!NRF_SAADC->EVENTS_CALIBRATEDONE);
            while(NRF_SAADC->STATUS == (SAADC_STATUS_STATUS_Busy << SAADC_STATUS_STATUS_Pos));
            LEDS_INVERT(BSP_LED_1_MASK);
            #ifdef UART_PRINTING_ENABLED
                NRF_LOG_INFO("SAADC Calibration complete! \r\n");
            #endif
        }

        #ifdef UART_PRINTING_ENABLED
            NRF_LOG_INFO("ADC event number: %d\r\n",(int)m_adc_evt_counter);                                      //Print the event number on UART
        #endif //UART_PRINTING_ENABLED
            for (int i = 0; i < SAADC_SAMPLES_IN_BUFFER; i++)
            {
                /*if(i % 2)
                {
                    m_rms_input_ain1[i/2] = p_event->data.done.p_buffer[i];
                    NRF_LOG_INFO("ain1 input = %d\r\n", p_event->data.done.p_buffer[i]);
                }
                else
                {*/
                    m_rms_input_ain0[i] = p_event->data.done.p_buffer[i];
                    //NRF_LOG_INFO("ain0 input = %d\r\n", p_event->data.done.p_buffer[i]);
               // }
                                                        
            }
            arm_rms_f32(m_rms_input_ain0, (uint32_t) SAADC_SAMPLES_IN_BUFFER, &m_rms_output_ain0);
            //arm_rms_f32(m_rms_input_ain1, (uint32_t) SAADC_SAMPLES_IN_BUFFER / 2, &m_rms_output_ain1);
            #ifdef UART_PRINTING_ENABLED
                NRF_LOG_INFO("RMS ain0: %d\r\n",(int)m_rms_output_ain0);
                //NRF_LOG_INFO("RMS ain1: %d\r\n",(int)m_rms_output_ain1);
             #endif //UART_PRINTING_ENABLED
            
            rms_value = (uint32_t)m_rms_output_ain0;

            if(rms_value != prev_rms_value)
            {
                rms_characteristic_update(&m_our_service, &rms_value);
            }
            prev_rms_value = rms_value;

            m_adc_evt_counter++;		

        nrf_drv_saadc_uninit();
        NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos);
        NVIC_ClearPendingIRQ(SAADC_IRQn);
        m_saadc_initialized = false;
    }
}

void saadc_init(void)
{
   ret_code_t err_code;
   nrf_drv_saadc_config_t saadc_config;

   nrf_saadc_channel_config_t channel_0_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
  // nrf_saadc_channel_config_t channel_1_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);

   saadc_config.resolution = NRF_SAADC_RESOLUTION_8BIT;
   saadc_config.oversample = SAADC_OVERSAMPLE;
   saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;

   err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
   APP_ERROR_CHECK(err_code);

   err_code = nrf_drv_saadc_channel_init(0, &channel_0_config);
   APP_ERROR_CHECK(err_code);

   //err_code = nrf_drv_saadc_channel_init(1, &channel_1_config);

   err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAADC_SAMPLES_IN_BUFFER);
   APP_ERROR_CHECK(err_code);

   err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAADC_SAMPLES_IN_BUFFER);
   APP_ERROR_CHECK(err_code);

}


int main(void)
{
#ifdef UART_PRINTING_ENABLED
    uint32_t err_code = NRF_LOG_INIT(timestamp);
    APP_ERROR_CHECK(err_code);                       //Configure Logging. LOGGING is used to show the SAADC sampled result. Default is UART, but RTT can be configured in sdk_config.h
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    NRF_LOG_INFO("\n\rSAADC Low Power Example.\r\n");	
#endif //UART_PRINTING_ENABLED	
    
    ppi_init();
    counter_timer_init();
    event_timer_init();

    APP_ERROR_CHECK(nrf_pwr_mgmt_init());
    LEDS_CONFIGURE(LEDS_MASK);                       //Configure all leds
    LEDS_OFF(LEDS_MASK);                             //Turn off all leds
	
    NRF_POWER->DCDCEN = 1;                           //Enabling the DCDC converter for lower current consumption
	
#ifdef UART_PRINTING_ENABLED
                                       //Configure UART. UART is used to show the SAADC sampled result.
    NRF_LOG_INFO("\n\rSAADC HAL simple example.\r\n");	
#endif //UART_PRINTING_ENABLED	

    lfclk_config();                                  //Configure low frequency 32kHz clock
    rtc_config();                                    //Configure RTC. The RTC will generate periodic interrupts. Requires 32kHz clock to operate.

    while(1)
    {
        
        nrf_pwr_mgmt_run();
        #ifdef UART_PRINTING_ENABLED
                NRF_LOG_FLUSH();
        #endif //UART_PRINTING_ENABLED
    }
}


/**
 * @}
 */
