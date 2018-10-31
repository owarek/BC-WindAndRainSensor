/*
Version: 1.1
Date: 31.10.2018
Info:
Měření rychlosti a směru větru společně se srážkoměrem. Naměřená data jsou navzorkována a poté každých 10 minut odeslána do BigClown hubu.
Kód převzat a upraven z https://github.com/hubmartin/bcf-sigfox-wind-station
Zapojení do sensor modulu:
Kanál A: anemometr (proti zemi)
Kanál B: korouhvička (proti zemi)
Kanál C: srážkoměr (proti zemi)
*/

#include <application.h>
#include <math.h>

#include "angle_average.h"

bc_led_t led;
bc_tag_temperature_t temperature_tag_internal;

float windSpeedAverage = 0;
float windSpeedMaximum = 0;
float windAngleAverage = 0;
float batteryVoltage = 0;

float rainMM = 0;              // mm srážek od posledního odeslání dat
float rainTotalMM = 0;         // mm srážek od zapnutí modulu
float mmInClick = 0.186267;    // mm srážek v jednom impulsu srážkoměru

bc_switch_t rain_gauge;

#define MAIN_TASK_PERIOD_SECONDS 15
#define TRANSMIT_PERIOD_MINUTES 10
#define WIND_DATA_STREAM_SAMPLES 40

// Data stream for wind speed averaging
BC_DATA_STREAM_FLOAT_BUFFER(stream_buffer_wind_speed, WIND_DATA_STREAM_SAMPLES)
bc_data_stream_t stream_wind_speed;

// Wind voltage table
// This was measured with original resistor in wind direction sensor and internal MCU pullup
// Some values are too close to each other. Better solution will be to use external 4k7 hardware pullup
// and edit this table to the real ADC values.
float windVoltageTable[16] = {
    1.463, // 0° North
    0.469, // 22.5°
    0.565, // 45°
    0.078, // 67.5°
    0.087, // 90° East
    0.062, // 112.5°
    0.179, // 135°
    0.118, // 157.5°
    0.298, // 180° South
    0.247, // 202.5°
    0.935, // 225°
    0.859, // 247.5°
    2.372, // 270° West
    1.650, // 292.5°
    1.970, // 315°
    1.152  // 337.5°
};

// Last angle and speed for debug
float windAngle;
float windSpeed;

void adc_event_handler(bc_adc_channel_t channel, bc_adc_event_t event, void *param);

// Wind data transmitting function
void publishWind(void *param)
{
    (void) param;

    //bc_led_pulse(&led, 500);

    int speed = (uint32_t)(windSpeedAverage * 10.0f);
    int speedMaximum = (uint32_t)(windSpeedMaximum * 10.0f);
    int angle = (uint32_t)windAngleAverage;
    
    // Publishing to related MQTT topics
    bc_radio_pub_int("meteo/speed",&speed);
    bc_radio_pub_int("meteo/speedMaximum",&speedMaximum);
    bc_radio_pub_int("meteo/angle",&angle);

    // Reset the maximum wind speed
    windSpeedMaximum = 0;

    // Register next publish in scheduler
    bc_scheduler_plan_current_relative(1000 * 60 * TRANSMIT_PERIOD_MINUTES);
}

// Rain and battery data transmitting function
void publishRain(void *param)
{
    (void) param;

    //bc_led_pulse(&led, 500);
    
    uint32_t battery = (uint32_t)(batteryVoltage * 1000);
    
    // Publishing to related MQTT topics
    bc_radio_pub_uint32("meteo/battery",&battery);
    bc_radio_pub_float("meteo/rain",&rainMM);
    bc_radio_pub_float("meteo/rainTotal",&rainTotalMM);

    // Reset rain
    rainMM = 0;

    // Register next publish in scheduler
    bc_scheduler_plan_current_relative(1000 * 60 * TRANSMIT_PERIOD_MINUTES);
}

float windVoltageToAngle(float voltage)
{
    float smallestDifferenceValue = 360.0f; // Set big number
    float smallestDifferenceAngle;

    uint32_t i;

    for(i = 0; i < 16; i++)
    {
        float currentDifference = fabs(voltage - windVoltageTable[i]);
        if(smallestDifferenceValue > currentDifference)
        {
            smallestDifferenceValue = currentDifference;
            smallestDifferenceAngle = 22.50f * i;
        }
    }

    return smallestDifferenceAngle;
}

float windAdcVoltage = 0;
uint16_t windAdc = 0;

void adc_event_handler(bc_adc_channel_t channel, bc_adc_event_t event, void *param)
{
    (void)param;
    (void)channel;

    if (event == BC_ADC_EVENT_DONE)
    {
        if(channel == BC_ADC_CHANNEL_A5)
        {
            #define ADC_VALUE_TO_VOLTAGE2(__RESULT__)   (((__RESULT__) * (0.00004743)))
            // Disable pullup
            bc_module_sensor_set_pull(BC_MODULE_SENSOR_CHANNEL_B, BC_MODULE_SENSOR_PULL_NONE);

            bc_adc_async_get_value(BC_ADC_CHANNEL_A5, &windAdc);
            windAdcVoltage = ADC_VALUE_TO_VOLTAGE2(windAdc);
            windAngle = windVoltageToAngle(windAdcVoltage);

            angle_average_add(windAngle);
            windAngleAverage = angle_average_get();

            //bc_radio_pub_float("meteo/debug/windVoltage",&windAdcVoltage);
            //uint32_t windAdcPub = (uint32_t)windAdc;
            //bc_radio_pub_uint32("meteo/debug/windVoltage",&windAdcPub);

            // Start battery measurement, default library collides sometimes with ADC measurement
            // so I use manual solution
            //_bc_module_battery_measurement(1);
            bc_gpio_set_output(BC_GPIO_P1, 1);
            bc_adc_async_measure(BC_ADC_CHANNEL_A0);
        }
        if(channel == BC_ADC_CHANNEL_A0)
        {
            #define ADC_VALUE_TO_VOLTAGE(__RESULT__)   (((__RESULT__) * (1 / 0.13)))
            uint16_t val;
            bc_adc_get_value(BC_ADC_CHANNEL_A0, &val);
            bc_gpio_set_output(BC_GPIO_P1, 0);

            batteryVoltage = ADC_VALUE_TO_VOLTAGE(val);

            //bc_radio_pub_float("meteo/debug/windVoltage",&batteryVoltage);
        }

    }
}

// Handler for rain gauge interrupt signals
void clickHandler(bc_switch_t *self, bc_switch_event_t event, void *event_param){
    if (event == BC_SWITCH_EVENT_OPENED)
    {
    }
    else if (event == BC_SWITCH_EVENT_CLOSED)
    {
    }
    rainTotalMM += mmInClick;
    rainMM += mmInClick;
}

void application_init(void)
{
    //bc_data_stream_init(&stream_wind_direction, 1/*WIND_DATA_STREAM_SAMPLES*/, &stream_buffer_wind_direction);
    bc_data_stream_init(&stream_wind_speed, 1/*WIND_DATA_STREAM_SAMPLES*/, &stream_buffer_wind_speed);

    bc_led_init(&led, BC_GPIO_LED, false, false);

    // Init internal temperature sensor to lower power sonsumption
    bc_tag_temperature_init(&temperature_tag_internal, BC_I2C_I2C0, BC_TAG_TEMPERATURE_I2C_ADDRESS_ALTERNATE);
    bc_tag_temperature_set_update_interval(&temperature_tag_internal, 5000 * 1000); // set big interval

    // Initialise radio
    bc_radio_init(BC_RADIO_MODE_NODE_SLEEPING);

    // Pulse counter
    bc_pulse_counter_init(BC_MODULE_SENSOR_CHANNEL_A, BC_PULSE_COUNTER_EDGE_FALL);
    bc_pulse_counter_set_event_handler(BC_MODULE_SENSOR_CHANNEL_A, NULL, NULL);

    // Rain counter
    bc_switch_init(&rain_gauge, BC_GPIO_P7, BC_SWITCH_TYPE_NC, BC_SWITCH_PULL_UP_DYNAMIC);
    bc_switch_set_event_handler(&rain_gauge, clickHandler, NULL);

    bc_module_sensor_set_mode(BC_MODULE_SENSOR_CHANNEL_B, BC_MODULE_SENSOR_MODE_INPUT);
    // Pullup is enabled in the task just before measuring
    //bc_module_sensor_set_pull(BC_MODULE_SENSOR_CHANNEL_B, BC_MODULE_SENSOR_PULL_UP_INTERNAL);

    // Initialize ADC channel - wind direction
    bc_adc_init();
    bc_adc_set_event_handler(BC_ADC_CHANNEL_A5, adc_event_handler, NULL);
    bc_adc_resolution_set(BC_ADC_CHANNEL_A5, BC_ADC_RESOLUTION_12_BIT);
    //bc_adc_oversampling_set(BC_ADC_CHANNEL_A5, BC_ADC_OVERSAMPLING_256);

    //bc_adc_set_event_handler(BC_ADC_CHANNEL_A0, adc_event_handler, NULL);
    //bc_adc_resolution_set(BC_ADC_CHANNEL_A0, BC_ADC_RESOLUTION_12_BIT);
    //bc_adc_oversampling_set(BC_ADC_CHANNEL_A0, BC_ADC_OVERSAMPLING_256);

    // Battery voltage
    bc_gpio_init(BC_GPIO_P1);
    bc_gpio_set_output(BC_GPIO_P1, 0);
    bc_gpio_set_mode(BC_GPIO_P1, BC_GPIO_MODE_OUTPUT);

    //bc_adc_init();
    bc_adc_set_event_handler(BC_ADC_CHANNEL_A0, adc_event_handler, NULL);

    // Publish scheduler is divided to two tasks (wind data and rain+battery data) one second between each.
    // When these two tasks are together it does not work properly.
    bc_scheduler_task_id_t publishWind_task_id = bc_scheduler_register(publishWind, NULL, 0);
    bc_scheduler_task_id_t publishRain_task_id = bc_scheduler_register(publishRain, NULL, 0);
    bc_scheduler_plan_relative(publishWind_task_id, 20 * 1000); // Schedule sending 20 seconds after start
    bc_scheduler_plan_relative(publishRain_task_id, 21 * 1000); // Schedule sending 20 seconds after start

    // Do a single blink to signalize that module is working
    bc_led_pulse(&led, 1000);
}

void application_task()
{
    // Enable pullup during wind direction measurement
    bc_module_sensor_set_pull(BC_MODULE_SENSOR_CHANNEL_B, BC_MODULE_SENSOR_PULL_UP_INTERNAL);
    bc_adc_async_measure(BC_ADC_CHANNEL_A5);
    
    // Reading and reseting wind speed counter
    float counter = (float)bc_pulse_counter_get(BC_MODULE_SENSOR_CHANNEL_A);
    bc_pulse_counter_reset(BC_MODULE_SENSOR_CHANNEL_A);

    // Get current wind speed, one pulse per second ~ 2.4kmph
    windSpeed = (counter / ((float)MAIN_TASK_PERIOD_SECONDS)) * 0.66666f; // 2.4km/h ~ 0.66666m/s

    // Save maximum wind speed value
    if(windSpeed > windSpeedMaximum)
    {
        windSpeedMaximum = windSpeed;
    }

    // Add value to stream and get average
    bc_data_stream_feed(&stream_wind_speed, &windSpeed);
    bc_data_stream_get_average(&stream_wind_speed, &windSpeedAverage);

    bc_scheduler_plan_current_relative(MAIN_TASK_PERIOD_SECONDS * 1000);
}
