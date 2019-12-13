/*
Version: 1.3
Date: 1.11.2018

Measurement of wind speed and direction together with rainfall sensor. Data are send every 10 minutes to the BigClown Hub

Original firmware https://github.com/hubmartin/bcf-sigfox-wind-station

Connection of the sensor module:
Channel A: anemometer (other contact connected to GND)
Channel B: wind direction (other contact connected to GND)
Channel C: rainfall switch (other contact connected to GND)

Datasheet
https://cdn.sparkfun.com/assets/8/4/c/d/6/Weather_Sensor_Assembly_Updated.pdf

Sparkfun
https://www.sparkfun.com/products/8942

Wether station items
http://www.meteo-pocasi.cz/eshop/nahradni-dily/meteostanice-me15/
https://www.conrad.cz/bezdratova-dcf-meteorologicka-stanice-renkforce-aok-5055.k1267773

Rainfall sensor Misol WH-SP-RG
Wind direction sensor MISOL WH-SP-WD
Wind speed sensor Misol WH-SP-WS01
Mounting arms
https://www.ebay.co.uk/itm/Mounting-arm-for-wind-speed-direction-sensor-spare-part-for-weather-station/272608358706?hash=item3f78b97132:g:86MAAOSw4A5Y2iBK&frcectupt=true
https://www.ebay.co.uk/itm/Mounting-arm-for-wind-speed-wind-direction-rain-meter-spare-part-for-weather/121918506255?hash=item1c62e8c50f:g:X1UAAOSwoBtW38co
*/

#include <application.h>
#include <math.h>

#include "angle_average.h"

bc_led_t led;
bc_tmp112_t temp;

float windSpeedAverage = 0;
float windSpeedMaximum = 0;
float windAngleAverage = 0;
float batteryVoltage = 0;

float rainMM = 0;              // mm of rainfall from last send data
float rainTotalMM = 0;         // mm of rainfall total

/*
The rainfall should be calibrated
Measure the area of the rainfall sensor in centimeters
inspiration by https://www.instructables.com/id/Arduino-Rain-Gauge-Calibration/

5 cm * 11 cm = 55 cm^2

Now measure the amount of water in mililiters to get one pulse.
It is better to start with bigger amount of water and then divide it with a number of pulses you get.
This averaging helps you get more precise numbers

In my test I need 1.6 ml to a single pulse from the sensor

1 ml = 1 cm^3

My sensor trips with 1.6mm

1.6 cm^3 / 55 cm^3 = 0.029090909.. cm = 0.29090 mm
*/

#define RAINFALL_PULSE_MM 0.29090f    // millimiters of rainfall in a single pulse from the sensor

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

//#define CLIMATE_ENABLE

// Climate module defines
#define UPDATE_NORMAL_INTERVAL             (10 * 1000)
#define BAROMETER_UPDATE_NORMAL_INTERVAL   (5 * 60 * 1000)

#define TEMPERATURE_TAG_PUB_NO_CHANGE_INTEVAL (15 * 60 * 1000)
#define TEMPERATURE_TAG_PUB_VALUE_CHANGE 0.2f

#define HUMIDITY_TAG_PUB_NO_CHANGE_INTEVAL (15 * 60 * 1000)
#define HUMIDITY_TAG_PUB_VALUE_CHANGE 5.0f

#define LUX_METER_TAG_PUB_NO_CHANGE_INTEVAL (15 * 60 * 1000)
#define LUX_METER_TAG_PUB_VALUE_CHANGE 25.0f

#define BAROMETER_TAG_PUB_NO_CHANGE_INTEVAL (15 * 60 * 1000)
#define BAROMETER_TAG_PUB_VALUE_CHANGE 20.0f

typedef struct
{
    uint8_t channel;
    float value;
    bc_tick_t next_pub;

} event_param_t;

struct {
    event_param_t temperature;
    event_param_t humidity;
    event_param_t illuminance;
    event_param_t pressure;
} params;

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
void publish_wind_task(void *param)
{
    (void) param;

    // Publishing to related MQTT topics
    bc_radio_pub_float("wind/speed/current",&windSpeedAverage);
    bc_radio_pub_float("wind/speed/maximal",&windSpeedMaximum);
    bc_radio_pub_float("wind/direction/degrees",&windAngleAverage);

    // Reset the maximum wind speed
    windSpeedMaximum = 0;

    // Register next publish in scheduler
    bc_scheduler_plan_current_relative(1000 * 60 * TRANSMIT_PERIOD_MINUTES);
}

// Rain and battery data transmitting function
void publish_rain_task(void *param)
{
    (void) param;

    //bc_led_pulse(&led, 500);

    bc_module_battery_get_voltage(&batteryVoltage);

    // Publishing to related MQTT topics
    if (bc_module_battery_get_format() == BC_MODULE_BATTERY_FORMAT_STANDARD)
    {
        bc_radio_pub_float("battery/standard/voltage",&batteryVoltage);
    }
    else if (bc_module_battery_get_format() == BC_MODULE_BATTERY_FORMAT_MINI)
    {
        bc_radio_pub_float("battery/mini/voltage",&batteryVoltage);
    }

    bc_radio_pub_float("thermometer/0:1/temperature", &internalTemperatureAverage);

    bc_radio_pub_float("rainfall/interval/mm", &rainMM);
    bc_radio_pub_float("rainfall/total/mm", &rainTotalMM);

    // Reset rain
    rainMM = 0;

    // Register next publish in scheduler
    bc_scheduler_plan_current_relative(1000 * 60 * TRANSMIT_PERIOD_MINUTES);
}

float windVoltageToAngle(float voltage)
{
    float smallestDifferenceValue = 1E8f; // Set big number
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

            //bc_log_debug("adc: %d, angle: %f", windAdc, windAngle);
        }
    }
}

// Handler for rain gauge interrupt signals
void rain_counter_handler(bc_switch_t *self, bc_switch_event_t event, void *event_param){
    if (event == BC_SWITCH_EVENT_OPENED)
    {
        rainTotalMM += RAINFALL_PULSE_MM;
        rainMM += RAINFALL_PULSE_MM;
    }
}


void climate_module_event_handler(bc_module_climate_event_t event, void *event_param)
{
    (void) event_param;

    float value;

    if (event == BC_MODULE_CLIMATE_EVENT_UPDATE_THERMOMETER)
    {
        if (bc_module_climate_get_temperature_celsius(&value))
        {
            if ((fabs(value - params.temperature.value) >= TEMPERATURE_TAG_PUB_VALUE_CHANGE) || (params.temperature.next_pub < bc_scheduler_get_spin_tick()))
            {
                bc_radio_pub_temperature(BC_RADIO_PUB_CHANNEL_R1_I2C0_ADDRESS_DEFAULT, &value);
                params.temperature.value = value;
                params.temperature.next_pub = bc_scheduler_get_spin_tick() + TEMPERATURE_TAG_PUB_NO_CHANGE_INTEVAL;
            }
        }
    }
    else if (event == BC_MODULE_CLIMATE_EVENT_UPDATE_HYGROMETER)
    {
        if (bc_module_climate_get_humidity_percentage(&value))
        {
            if ((fabs(value - params.humidity.value) >= HUMIDITY_TAG_PUB_VALUE_CHANGE) || (params.humidity.next_pub < bc_scheduler_get_spin_tick()))
            {
                bc_radio_pub_humidity(BC_RADIO_PUB_CHANNEL_R3_I2C0_ADDRESS_DEFAULT, &value);
                params.humidity.value = value;
                params.humidity.next_pub = bc_scheduler_get_spin_tick() + HUMIDITY_TAG_PUB_NO_CHANGE_INTEVAL;
            }
        }
    }
    else if (event == BC_MODULE_CLIMATE_EVENT_UPDATE_LUX_METER)
    {
        if (bc_module_climate_get_illuminance_lux(&value))
        {
            if (value < 1)
            {
                value = 0;
            }
            if ((fabs(value - params.illuminance.value) >= LUX_METER_TAG_PUB_VALUE_CHANGE) || (params.illuminance.next_pub < bc_scheduler_get_spin_tick()) ||
                    ((value == 0) && (params.illuminance.value != 0)) || ((value > 1) && (params.illuminance.value == 0)))
            {
                bc_radio_pub_luminosity(BC_RADIO_PUB_CHANNEL_R1_I2C0_ADDRESS_DEFAULT, &value);
                params.illuminance.value = value;
                params.illuminance.next_pub = bc_scheduler_get_spin_tick() + LUX_METER_TAG_PUB_NO_CHANGE_INTEVAL;
            }
        }
    }
    else if (event == BC_MODULE_CLIMATE_EVENT_UPDATE_BAROMETER)
    {
        if (bc_module_climate_get_pressure_pascal(&value))
        {
            if ((fabs(value - params.pressure.value) >= BAROMETER_TAG_PUB_VALUE_CHANGE) || (params.pressure.next_pub < bc_scheduler_get_spin_tick()))
            {
                float meter;

                if (!bc_module_climate_get_altitude_meter(&meter))
                {
                    return;
                }

                bc_radio_pub_barometer(BC_RADIO_PUB_CHANNEL_R1_I2C0_ADDRESS_DEFAULT, &value, &meter);
                params.pressure.value = value;
                params.pressure.next_pub = bc_scheduler_get_spin_tick() + BAROMETER_TAG_PUB_NO_CHANGE_INTEVAL;
            }
        }
    }
}

void application_init(void)
{
    bc_data_stream_init(&stream_wind_speed, 1, &stream_buffer_wind_speed);

    //bc_log_init(BC_LOG_LEVEL_DEBUG, BC_LOG_TIMESTAMP_ABS);

    //init temperature buffer stream
    bc_data_stream_init(&stream_internal_temperature, 1, &stream_buffer_internal_temperature);

    bc_led_init(&led, BC_GPIO_LED, false, false);

    // Init internal temperature sensor to lower power sonsumption
    bc_tmp112_init(&temp, BC_I2C_I2C0, 0x49);

    // set measurement handler (call "tmp112_event_handler()" after measurement)
    bc_tmp112_set_event_handler(&temp, tmp112_event_handler, NULL);

    // Pulse counter
    bc_pulse_counter_init(BC_MODULE_SENSOR_CHANNEL_A, BC_PULSE_COUNTER_EDGE_FALL);
    bc_pulse_counter_set_event_handler(BC_MODULE_SENSOR_CHANNEL_A, NULL, NULL);

    // Rain counter
    bc_switch_init(&rain_gauge, BC_GPIO_P7, BC_SWITCH_TYPE_NC, BC_SWITCH_PULL_UP_DYNAMIC);
    bc_switch_set_event_handler(&rain_gauge, rain_counter_handler, NULL);

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
    bc_scheduler_task_id_t publish_wind_task_id = bc_scheduler_register(publish_wind_task, NULL, 0);
    bc_scheduler_task_id_t publish_rain_task_id = bc_scheduler_register(publish_rain_task, NULL, 0);
    bc_scheduler_plan_relative(publish_wind_task_id, 20 * 1000); // Schedule sending 20 seconds after start
    bc_scheduler_plan_relative(publish_rain_task_id, 21 * 1000); // Schedule sending 21 seconds after start

    #ifdef CLIMATE_ENABLE
    // Initialize climate module
    bc_module_climate_init();
    bc_module_climate_set_event_handler(climate_module_event_handler, NULL);
    bc_module_climate_set_update_interval_thermometer(UPDATE_NORMAL_INTERVAL);
    bc_module_climate_set_update_interval_hygrometer(UPDATE_NORMAL_INTERVAL);
    bc_module_climate_set_update_interval_lux_meter(UPDATE_NORMAL_INTERVAL);
    bc_module_climate_set_update_interval_barometer(BAROMETER_UPDATE_NORMAL_INTERVAL);
    bc_module_climate_measure_all_sensors();
    #endif

    // Initialize radio
    bc_radio_init(BC_RADIO_MODE_NODE_SLEEPING);
    bc_radio_pairing_request("wind-station", VERSION);

    // Do a single blink to signalize that module is working
    bc_led_pulse(&led, 2000);
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

    //bc_log_debug("speed %f", windSpeed);

    bc_scheduler_plan_current_relative(MAIN_TASK_PERIOD_SECONDS * 1000);
}
