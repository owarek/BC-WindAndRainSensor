/*
Version: 1.3
Date: 1.11.2018
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
//bc_tag_temperature_t temperature_tag_internal;
bc_tmp112_t temp;


float windSpeedAverage = 0;
float windSpeedMaximum = 0;
float windAngleAverage = 0;
float batteryVoltage = 0;

float rainMM = 0;              // mm srážek od posledního odeslání dat
float rainTotalMM = 0;         // mm srážek od zapnutí modulu
float mmInClick = 0.186267;    // mm srážek v jednom impulsu srážkoměru

float internalTemperature = 0; 
float internalTemperatureAverage = 0;
bc_switch_t rain_gauge;

#define MAIN_TASK_PERIOD_SECONDS 15
#define TRANSMIT_PERIOD_MINUTES 10
#define WIND_DATA_STREAM_SAMPLES 40

// Data stream for wind speed averaging
BC_DATA_STREAM_FLOAT_BUFFER(stream_buffer_wind_speed, WIND_DATA_STREAM_SAMPLES)
BC_DATA_STREAM_FLOAT_BUFFER(stream_buffer_internal_temperature, WIND_DATA_STREAM_SAMPLES)
bc_data_stream_t stream_wind_speed;
bc_data_stream_t stream_internal_temperature;

// Wind voltage table
// This was measured with original resistor in wind direction sensor and internal MCU pullup
// Some values are too close to each other. Better solution will be to use external 4k7 hardware pullup
// and edit this table to the real ADC values.
float windADCTable[16] = {
    30845, // 0° North
    9888, // 22.5°
    11912, // 45°
    1645, // 67.5°
    1834, // 90° East
    1307, // 112.5°
    3774, // 135°
    2488, // 157.5°
    6283, // 180° South
    5208, // 202.5°
    19713, // 225°
    18111, // 247.5°
    50011, // 270° West
    34788, // 292.5°
    41535, // 315°
    24288  // 337.5°
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

    bc_module_battery_get_voltage(&batteryVoltage);
    
    // Publishing to related MQTT topics
    bc_radio_pub_float("meteo/battery",&batteryVoltage);
    bc_radio_pub_float("meteo/internalTemperature",&internalTemperatureAverage);

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
        float currentDifference = fabs(voltage - windADCTable[i]);
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

void tmp112_event_handler(bc_tmp112_t *self, bc_tmp112_event_t event, void *event_param)
{
    (void) self;
    (void) event_param;

    if (event == BC_TMP112_EVENT_UPDATE)
    {
        bc_tmp112_get_temperature_celsius(&temp, &internalTemperature);
    }
}

void adc_event_handler(bc_adc_channel_t channel, bc_adc_event_t event, void *param)
{
    (void)param;
    (void)channel;

    if (event == BC_ADC_EVENT_DONE)
    {
        if(channel == BC_ADC_CHANNEL_A5)
        {
            //#define ADC_VALUE_TO_VOLTAGE2(__RESULT__)   (((__RESULT__) * (0.00004743)))

            // Disable pullup
            bc_module_sensor_set_pull(BC_MODULE_SENSOR_CHANNEL_B, BC_MODULE_SENSOR_PULL_NONE);

            bc_adc_async_get_value(BC_ADC_CHANNEL_A5, &windAdc);
            //windAdcVoltage = ADC_VALUE_TO_VOLTAGE2(windAdc);
            windAngle = windVoltageToAngle(windAdc);

            angle_average_add(windAngle);
            windAngleAverage = angle_average_get();

            // Start battery measurement, default library collides sometimes with ADC measurement
            bc_module_battery_measure();
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

    //init temperature buffer stream
    bc_data_stream_init(&stream_internal_temperature, 1/*WIND_DATA_STREAM_SAMPLES*/, &stream_buffer_internal_temperature);

    bc_led_init(&led, BC_GPIO_LED, false, false);

    // Init internal temperature sensor to lower power sonsumption
    bc_tmp112_init(&temp, BC_I2C_I2C0, 0x49);

    // set measurement handler (call "tmp112_event_handler()" after measurement)
    bc_tmp112_set_event_handler(&temp, tmp112_event_handler, NULL);

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

    // Initialize ADC channel - wind direction
    bc_adc_init();
    bc_adc_set_event_handler(BC_ADC_CHANNEL_A5, adc_event_handler, NULL);
    bc_adc_resolution_set(BC_ADC_CHANNEL_A5, BC_ADC_RESOLUTION_12_BIT);

    // Battery module voltage
    bc_module_battery_init();

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

    // Read temperature and save it to the buffer stream
    bc_tmp112_measure(&temp);   
    bc_data_stream_feed(&stream_internal_temperature, &internalTemperature);
    bc_data_stream_get_average(&stream_internal_temperature, &internalTemperatureAverage);

    bc_scheduler_plan_current_relative(MAIN_TASK_PERIOD_SECONDS * 1000);
}
