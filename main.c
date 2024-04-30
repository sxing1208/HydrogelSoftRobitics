/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
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
/** @file
 *
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "sensorsim.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_drv_saadc.h"
#include "nrfx_saadc.h"

#include "nrf_drv_pwm.h"
#include "app_util_platform.h"
#include "boards.h"
#include "bsp.h"
#include "nrf_drv_clock.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ble_bas.h"
#include "ble_dis.h"
#include "ble_cus.h"

#include "nrf_delay.h"

#define DEVICE_NAME                     "Gait Analyzer"                        /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "Bai Lab at UNC"                        /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUMBER                    "v 0.3.2"                               /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION                18000                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */
#define Channel Number                  2                                       /**< Number of analog channels being sampled**/

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define BATTERY_LEVEL_MEAS_INTERVAL         APP_TIMER_TICKS(10000)              /**< Battery level measurement interval (ticks). */
#define NOTIFICATION_INTERVAL               APP_TIMER_TICKS(1000)              /**< Battery level measurement interval (ticks). */


NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);  
APP_TIMER_DEF(m_battery_timer_id); 
APP_TIMER_DEF(m_notification_timer_id);                                     

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);

static uint16_t const              m_demo1_top  = 10000;
static uint16_t const              m_demo1_step = 1;
static uint8_t                     m_demo1_phase;
static nrf_pwm_values_individual_t m_demo1_seq_values;
static nrf_pwm_sequence_t    m_demo1_seq =
{
    .values.p_individual = &m_demo1_seq_values,
    .length              = NRF_PWM_VALUES_LENGTH(m_demo1_seq_values),
    .repeats             = 0,
    .end_delay           = 0
};
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

//storing the SAADC readout value
static nrf_saadc_value_t adc_value;
static nrf_saadc_channel_config_t m_config_0 = 
{
  .resistor_p = NRF_SAADC_RESISTOR_DISABLED, 
 .resistor_n = NRF_SAADC_RESISTOR_DISABLED, 
 .gain = NRF_SAADC_GAIN1_4, 
 .reference = NRF_SAADC_REFERENCE_VDD4, 
 .acq_time = NRF_SAADC_ACQTIME_10US, 
 .mode = NRF_SAADC_MODE_SINGLE_ENDED, 
 .burst = NRF_SAADC_BURST_DISABLED, 
 .pin_p = NRF_SAADC_INPUT_AIN0, 
 .pin_n = NRF_SAADC_INPUT_DISABLED 
};
static nrf_saadc_channel_config_t m_config_1 = 
{
  .resistor_p = NRF_SAADC_RESISTOR_DISABLED, 
 .resistor_n = NRF_SAADC_RESISTOR_DISABLED, 
 .gain = NRF_SAADC_GAIN1_4, 
 .reference = NRF_SAADC_REFERENCE_VDD4, 
 .acq_time = NRF_SAADC_ACQTIME_10US, 
 .mode = NRF_SAADC_MODE_SINGLE_ENDED, 
 .burst = NRF_SAADC_BURST_DISABLED, 
 .pin_p = NRF_SAADC_INPUT_AIN1, 
 .pin_n = NRF_SAADC_INPUT_DISABLED 
};
static nrf_saadc_channel_config_t m_config_2 = 
{
  .resistor_p = NRF_SAADC_RESISTOR_DISABLED, 
 .resistor_n = NRF_SAADC_RESISTOR_DISABLED, 
 .gain = NRF_SAADC_GAIN1_4, 
 .reference = NRF_SAADC_REFERENCE_VDD4, 
 .acq_time = NRF_SAADC_ACQTIME_10US, 
 .mode = NRF_SAADC_MODE_SINGLE_ENDED, 
 .burst = NRF_SAADC_BURST_DISABLED, 
 .pin_p = NRF_SAADC_INPUT_AIN2, 
 .pin_n = NRF_SAADC_INPUT_DISABLED 
};
static nrf_saadc_channel_config_t m_config_3 = 
{
  .resistor_p = NRF_SAADC_RESISTOR_DISABLED, 
 .resistor_n = NRF_SAADC_RESISTOR_DISABLED, 
 .gain = NRF_SAADC_GAIN1_4, 
 .reference = NRF_SAADC_REFERENCE_VDD4, 
 .acq_time = NRF_SAADC_ACQTIME_10US, 
 .mode = NRF_SAADC_MODE_SINGLE_ENDED, 
 .burst = NRF_SAADC_BURST_DISABLED, 
 .pin_p = NRF_SAADC_INPUT_AIN3, 
 .pin_n = NRF_SAADC_INPUT_DISABLED 
};
static nrf_saadc_channel_config_t m_config_4 = 
{
  .resistor_p = NRF_SAADC_RESISTOR_DISABLED, 
 .resistor_n = NRF_SAADC_RESISTOR_DISABLED, 
 .gain = NRF_SAADC_GAIN1_4, 
 .reference = NRF_SAADC_REFERENCE_VDD4, 
 .acq_time = NRF_SAADC_ACQTIME_10US, 
 .mode = NRF_SAADC_MODE_SINGLE_ENDED, 
 .burst = NRF_SAADC_BURST_DISABLED, 
 .pin_p = NRF_SAADC_INPUT_AIN4, 
 .pin_n = NRF_SAADC_INPUT_DISABLED 
};
static nrf_saadc_channel_config_t m_config_5 = 
{
  .resistor_p = NRF_SAADC_RESISTOR_DISABLED, 
 .resistor_n = NRF_SAADC_RESISTOR_DISABLED, 
 .gain = NRF_SAADC_GAIN1_4, 
 .reference = NRF_SAADC_REFERENCE_VDD4, 
 .acq_time = NRF_SAADC_ACQTIME_10US, 
 .mode = NRF_SAADC_MODE_SINGLE_ENDED, 
 .burst = NRF_SAADC_BURST_DISABLED, 
 .pin_p = NRF_SAADC_INPUT_AIN5, 
 .pin_n = NRF_SAADC_INPUT_DISABLED 
};
static nrf_saadc_channel_config_t m_config_6 = 
{
  .resistor_p = NRF_SAADC_RESISTOR_DISABLED, 
 .resistor_n = NRF_SAADC_RESISTOR_DISABLED, 
 .gain = NRF_SAADC_GAIN1_4, 
 .reference = NRF_SAADC_REFERENCE_VDD4, 
 .acq_time = NRF_SAADC_ACQTIME_10US, 
 .mode = NRF_SAADC_MODE_SINGLE_ENDED, 
 .burst = NRF_SAADC_BURST_DISABLED, 
 .pin_p = NRF_SAADC_INPUT_AIN6, 
 .pin_n = NRF_SAADC_INPUT_DISABLED 
};
static nrf_saadc_channel_config_t m_config_7 = 
{
  .resistor_p = NRF_SAADC_RESISTOR_DISABLED, 
 .resistor_n = NRF_SAADC_RESISTOR_DISABLED, 
 .gain = NRF_SAADC_GAIN1_4, 
 .reference = NRF_SAADC_REFERENCE_VDD4, 
 .acq_time = NRF_SAADC_ACQTIME_10US, 
 .mode = NRF_SAADC_MODE_SINGLE_ENDED, 
 .burst = NRF_SAADC_BURST_DISABLED, 
 .pin_p = NRF_SAADC_INPUT_AIN7, 
 .pin_n = NRF_SAADC_INPUT_DISABLED 
};

//Interval Settings for bipolar electrical stimulations
static uint8_t m_interval_count = 0;
static uint8_t m_max_interval =10;
static uint8_t m_pulse_width = 20;
static int m_pwm_value = 2500;

//set custom service value
static uint16_t m_custom_value = 0;

volatile bool flag = 1;

static uint8_t m_SAADC_channel_count = 0;

#define channel_length 8

char* arrayToString(uint16_t a[channel_length]) {
    // Each number can be up to 5 characters, plus one for the comma, times the channel length
    // Minus one since the last number does not need a comma, plus one for null terminator
    static char str[channel_length * 6];
    str[0] = '\0'; // Initialize the string to empty

    for(int i = 0; i < channel_length; i++) {
        char temp[6]; // Temporary string for the current number
        snprintf(temp, sizeof(temp), "%05u", a[i]);
        strcat(str, temp); // Add the current number to the final string
        if (i < channel_length - 1) {
            strcat(str, ","); // Add a comma after the number if it's not the last one
        }
    }

    return str;
}

//saadc init
static void SAADC_callback(nrfx_saadc_evt_t const * p_event) //Callback for SAADC
{
  //empty callback
}

//SAADC multi_channel reader
static void SAADC_multichannel_init() //initialize SAADC multiple channel readout
{
  nrfx_saadc_config_t m_saadc_config =
  {
    .resolution = NRF_SAADC_RESOLUTION_14BIT,
    .oversample = NRF_SAADC_OVERSAMPLE_DISABLED,
    .interrupt_priority = NRFX_SAADC_CONFIG_IRQ_PRIORITY,
    .low_power_mode = NRFX_SAADC_CONFIG_LP_MODE
  };
  nrf_saadc_channel_config_t channel_config_0 =
    m_config_0;
  nrf_saadc_channel_config_t channel_config_1 =
    m_config_1;
  nrf_saadc_channel_config_t channel_config_2 =
    m_config_2;
  nrf_saadc_channel_config_t channel_config_3 =
    m_config_3;
  nrf_saadc_channel_config_t channel_config_4 =
    m_config_4;
  nrf_saadc_channel_config_t channel_config_5 =
    m_config_5;
  nrf_saadc_channel_config_t channel_config_6 =
    m_config_6;
  nrf_saadc_channel_config_t channel_config_7 =
    m_config_7;
  APP_ERROR_CHECK(nrfx_saadc_init(&m_saadc_config,SAADC_callback));
  //initialize channels
  APP_ERROR_CHECK(nrfx_saadc_channel_init(0,&channel_config_0));
  APP_ERROR_CHECK(nrfx_saadc_channel_init(1,&channel_config_1));
  APP_ERROR_CHECK(nrfx_saadc_channel_init(2,&channel_config_2));
  APP_ERROR_CHECK(nrfx_saadc_channel_init(3,&channel_config_3));
  APP_ERROR_CHECK(nrfx_saadc_channel_init(4,&channel_config_4));
  APP_ERROR_CHECK(nrfx_saadc_channel_init(5,&channel_config_5));
  APP_ERROR_CHECK(nrfx_saadc_channel_init(6,&channel_config_6));
  APP_ERROR_CHECK(nrfx_saadc_channel_init(7,&channel_config_7));
}

/* YOUR_JOB: Declare all services structure your application is using
 *  BLE_XYZ_DEF(m_xyz);
 */
BLE_BAS_DEF(m_bas);
BLE_CUS_DEF(m_cus);

// YOUR_JOB: Use UUIDs for service(s) used in your application.
static ble_uuid_t m_adv_uuids[] =                                               /**< Universally unique service identifiers. */
{
    {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE}
};

static void pwm_init()     //initialize pulse widith modulation (pwm) in nrfx for pseudo-analog output
{
    nrf_drv_pwm_config_t const config0 =
    {
        .output_pins =
        {
            //your job: declare which GPIO pin you want to use as a pwm output channel
            BSP_LED_3,         // channel 0
            ////BSP_LED_1 , // channel 1
            ////BSP_LED_2 , // channel 2
            ////BSP_LED_3
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_16MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = m_demo1_top,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &config0, NULL)); //initialize nrf pwm driver
    printf("PWM initialized."); //for debugging
}
static void pwm()
{
    NRF_LOG_INFO("PWM start");

    //your job: declare the default values for each output channel.

    nrf_gpio_pin_write(18,0);
    nrf_gpio_pin_write(19,0);
    m_demo1_seq_values.channel_0 = m_pwm_value;
    //m_demo1_seq_values.channel_1 = 10000;
    //m_demo1_seq_values.channel_2 = 10000;
    //m_demo1_seq_values.channel_3 = 10000;
    m_demo1_phase                = 0;

    (void)nrf_drv_pwm_simple_playback(&m_pwm0, &m_demo1_seq, 1,
                                      NRF_DRV_PWM_FLAG_LOOP); //pwm set to play in a loop manner
    printf("PWM started.");
}
// --------------starting here are communication functions for BLE-----------
static void advertising_start(bool erase_bonds);


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;

        default:
            break;
    }
}

static void battery_level_update(void)
{
    ///*brief function for updating "battery level" for BLE service and updating pwm output */
    //ret_code_t err_code;
    //uint8_t  battery_level = 0; 
    ////nrfx_saadc_sample_convert(1,&adc_value);
    ////battery_level = adc_value*100/16000;  //convert saadc readout to a 8 bit integer
    //err_code = ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
    //printf("update");
    //if ((err_code != NRF_SUCCESS) &&
    //    (err_code != NRF_ERROR_INVALID_STATE) &&
    //    (err_code != NRF_ERROR_RESOURCES) &&
    //    (err_code != NRF_ERROR_BUSY) &&
    //    (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    //   )
    //{
    //    APP_ERROR_HANDLER(err_code);
    //}
    //empty callback
}

/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    printf("timeout!");
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}

/**@brief Function for handling the Custom Value measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void notification_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    ret_code_t err_code;
    
    // Increment the value of m_custom_value before nortifing it.
    uint16_t read_value[8];
    for(int i=0;i<=7;i++)
    {
      //nrf_gpio_pin_toggle(19);
      //nrf_delay_ms(5);
      err_code = nrfx_saadc_sample_convert(i, &read_value[i]); //read impedance
      if(err_code != NRF_SUCCESS)
      {
          APP_ERROR_HANDLER(err_code);
      }
      //nrf_delay_ms(m_pulse_width-5);
      //nrf_gpio_pin_toggle(19);
      //nrf_gpio_pin_toggle(18);
      //nrf_delay_ms(m_pulse_width*2);
      //nrf_gpio_pin_toggle(18);
      //if(read_value>=9000)
      //{
      //    m_demo1_seq_values.channel_0 = 2500;
      //    m_max_interval = 5;
      //    m_pulse_width = 40;
      //}
      //else
      //      {
      //         m_demo1_seq_values.channel_0 = 5000;
      //         m_max_interval = 10;
      //         m_pulse_width = 20;
      //      }
     //Update Impedance Value
     
     if(read_value[i]<0)
     {
        read_value[i] = 0;
     }

    }
      err_code = ble_cus_custom_value_update(&m_cus, arrayToString(read_value));
      NRF_LOG_INFO("%s",arrayToString(read_value))
      printf("update");
      if ((err_code != NRF_SUCCESS) &&
          (err_code != NRF_ERROR_INVALID_STATE) &&
          (err_code != NRF_ERROR_RESOURCES) &&
          (err_code != NRF_ERROR_BUSY) &&
          (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
         )
          {
              APP_ERROR_HANDLER(err_code);
          }


}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.

    /* YOUR_JOB: Create any timers to be used by the application.
                 Below is an example of how to create a timer.
                 For every new timer needed, increase the value of the macro APP_TIMER_MAX_TIMERS by
                 one.
     */
       err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
       err_code = app_timer_create(&m_notification_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                notification_timeout_handler);
       APP_ERROR_CHECK(err_code); 
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    /* YOUR_JOB: Use an appearance value matching the application's use case.
       err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
       APP_ERROR_CHECK(err_code); */

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the YYY Service events.
 * YOUR_JOB implement a service handler function depending on the event the service you are using can generate
 *
 * @details This function will be called for all YY Service events which are passed to
 *          the application.
 *
 * @param[in]   p_yy_service   YY Service structure.
 * @param[in]   p_evt          Event received from the YY Service.
 *
 *
static void on_yys_evt(ble_yy_service_t     * p_yy_service,
                       ble_yy_service_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_YY_NAME_EVT_WRITE:
            APPL_LOG("[APPL]: charact written with value %s. ", p_evt->params.char_xx.value.p_str);
            break;
        default:
            // No implementation needed.
            break;
    }
}
*/
/**@brief Function for handling the Custom Service Service events.
 *
 * @details This function will be called for all Custom Service events which are passed to
 *          the application.
 *
 * @param[in]   p_cus_service  Custom Service structure.
 * @param[in]   p_evt          Event received from the Custom Service.
 *
 */
static void on_cus_evt(ble_cus_t     * p_cus_service,
                       ble_cus_evt_t * p_evt)
{
    ret_code_t err_code;
    
    switch(p_evt->evt_type)
    {
        case BLE_CUS_EVT_NOTIFICATION_ENABLED:
            
             //err_code = app_timer_start(m_notification_timer_id, NOTIFICATION_INTERVAL, NULL);
             APP_ERROR_CHECK(err_code);
             break;

        case BLE_CUS_EVT_NOTIFICATION_DISABLED:

            //err_code = app_timer_stop(m_notification_timer_id);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_CUS_EVT_CONNECTED:
            break;

        case BLE_CUS_EVT_DISCONNECTED:
              break;

        default:
              // No implementation needed.
              break;
    }
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    /* YOUR_JOB: Add code to initialize the services used by the application.
       ble_xxs_init_t                     xxs_init;
       ble_yys_init_t                     yys_init;*/
      ble_bas_init_t     bas_init;
      ble_dis_init_t     dis_init;
      ble_cus_init_t     cus_init;
       // Initialize XXX Service.
      memset(&bas_init, 0, sizeof(bas_init));
      bas_init.evt_handler          = NULL;
      bas_init.support_notification = true;
      bas_init.p_report_ref         = NULL;
      bas_init.initial_batt_level   = 100;
      
    // Here the sec level for the Battery Service can be changed/increased.
    bas_init.bl_rd_sec        = SEC_OPEN;
    bas_init.bl_cccd_wr_sec   = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str, (char *)MODEL_NUMBER);


    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);

    memset(&cus_init, 0, sizeof(cus_init));
	
        cus_init.evt_handler                = on_cus_evt;
    
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init.custom_value_char_attr_md.cccd_write_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init.custom_value_char_attr_md.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init.custom_value_char_attr_md.write_perm);
    
        err_code = ble_cus_init(&m_cus, &cus_init);
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
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
    ret_code_t             err_code;
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


/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
    ret_code_t err_code;
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    err_code = app_timer_start(m_notification_timer_id, NOTIFICATION_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
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
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            printf("Disconnected");
            //connectivity_bool = 0;
            // LED indication will be changed when advertising starts.
            break;

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            printf("Connected");
            //connectivity_bool = 1;
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break; // BSP_EVENT_SLEEP

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break; // BSP_EVENT_DISCONNECT

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break; // BSP_EVENT_KEY_0

        default:
            break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */



/**@brief Function for application main entry.
 */
int main(void)
{
    
    
    bool erase_bonds;

    // Initialize.
    pwm_init();
    log_init();
    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();
    conn_params_init();
    peer_manager_init();
    SAADC_multichannel_init();

    // Start execution.
    NRF_LOG_INFO("Template example started.");
    application_timers_start();
    advertising_start(erase_bonds);
    pwm();

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}
