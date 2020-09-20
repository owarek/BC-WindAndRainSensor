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

#define MEASURE_INTERVAL_SECONDS 15
#define TRANSMIT_PERIOD_MINUTES  10
#define WIND_DATA_STREAM_SAMPLES 40
#define BATTERY_UPDATE_INTERVAL (60 * 60 * 1000)

// Data stream for wind speed averaging
BC_DATA_STREAM_FLOAT_BUFFER(stream_buffer_wind_speed, WIND_DATA_STREAM_SAMPLES)
BC_DATA_STREAM_FLOAT_BUFFER(stream_buffer_internal_temperature, WIND_DATA_STREAM_SAMPLES)
bc_data_stream_t stream_wind_speed;
bc_data_stream_t stream_internal_temperature;

#define CLIMATE_ENABLE

// Climate module defines
#define TEMPERATURE_TRANSMIT_VALUE_DIFF 1.0f
#define HUMIDITY_TRANSMIT_VALUE_DIFF 5.0f

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

            bc_log_debug("adc: %d, angle: %f windAngleAverage: %f", windAdc, windAngle, windAngleAverage);
        }
    }
}

void battery_event_handler(bc_module_battery_event_t event, void *event_param)
{
    if (event == BC_MODULE_BATTERY_EVENT_UPDATE)
    {
        // Read battery voltage
        if (!bc_module_battery_get_voltage(&batteryVoltage))
        {
            return;
        }

        // Publishing to related MQTT topics
        if (bc_module_battery_get_format() == BC_MODULE_BATTERY_FORMAT_STANDARD)
        {
            bc_radio_pub_float("battery/standard/voltage",&batteryVoltage);
        }
        else if (bc_module_battery_get_format() == BC_MODULE_BATTERY_FORMAT_MINI)
        {
            bc_radio_pub_float("battery/mini/voltage",&batteryVoltage);
        }
    }
}

void climate_module_event_handler(bc_module_climate_event_t event, void *event_param)
{
    (void) event_param;

    if (event == BC_MODULE_CLIMATE_EVENT_UPDATE_THERMOMETER)
    {
        static struct
        {
            float temperature;
            bc_tick_t next_pub;
        } radio_pub = { .next_pub = 0 };

        float temperature;

        if (bc_module_climate_get_temperature_celsius(&temperature))
        {
            if ((radio_pub.next_pub < bc_scheduler_get_spin_tick()) || (fabs(temperature - radio_pub.temperature) >= TEMPERATURE_TRANSMIT_VALUE_DIFF))
            {
                bc_radio_pub_temperature(BC_RADIO_PUB_CHANNEL_R1_I2C0_ADDRESS_DEFAULT, &temperature);
                radio_pub.temperature = temperature;
                radio_pub.next_pub = bc_scheduler_get_spin_tick() + (TRANSMIT_PERIOD_MINUTES * 60 * 1000);
            }
        }
    }
    else if (event == BC_MODULE_CLIMATE_EVENT_UPDATE_HYGROMETER)
    {
        static struct
        {
            float humidity;
            bc_tick_t next_pub;
        } radio_pub = { .next_pub = 0 };

        float humidity;

        if (bc_module_climate_get_humidity_percentage(&humidity))
        {
            if ((radio_pub.next_pub < bc_scheduler_get_spin_tick()) || (fabs(humidity - radio_pub.humidity) >= HUMIDITY_TRANSMIT_VALUE_DIFF))
            {
                bc_radio_pub_humidity(BC_RADIO_PUB_CHANNEL_R3_I2C0_ADDRESS_DEFAULT, &humidity);
                radio_pub.humidity = humidity;
                radio_pub.next_pub = bc_scheduler_get_spin_tick() + (TRANSMIT_PERIOD_MINUTES * 60 * 1000);
            }
        }
    }
    else if (event == BC_MODULE_CLIMATE_EVENT_UPDATE_LUX_METER)
    {
        float illuminance;

        if (bc_module_climate_get_illuminance_lux(&illuminance))
        {
            bc_radio_pub_luminosity(BC_RADIO_PUB_CHANNEL_R1_I2C0_ADDRESS_DEFAULT, &illuminance);
        }
    }
    else if (event == BC_MODULE_CLIMATE_EVENT_UPDATE_BAROMETER)
    {
        float pressure;
        float meter;

        if (bc_module_climate_get_pressure_pascal(&pressure) && bc_module_climate_get_altitude_meter(&meter))
        {
            bc_radio_pub_barometer(BC_RADIO_PUB_CHANNEL_R1_I2C0_ADDRESS_DEFAULT, &pressure, &meter);
        }
    }
}

void application_init(void)
{
    bc_data_stream_init(&stream_wind_speed, 1, &stream_buffer_wind_speed);

    bc_log_init(BC_LOG_LEVEL_OFF, BC_LOG_TIMESTAMP_ABS);

    //init temperature buffer stream
    bc_data_stream_init(&stream_internal_temperature, 1, &stream_buffer_internal_temperature);

    bc_led_init(&led, BC_GPIO_LED, false, false);

    // Init internal temperature sensor to lower power sonsumption
    bc_tmp112_init(&temp, BC_I2C_I2C0, 0x49);
    // set measurement handler (call "tmp112_event_handler()" after measurement)
    bc_tmp112_set_event_handler(&temp, tmp112_event_handler, NULL);

    // Pulse counter
    bc_pulse_counter_init(BC_MODULE_SENSOR_CHANNEL_A, BC_PULSE_COUNTER_EDGE_FALL);

    // Rain counter
    bc_pulse_counter_init(BC_MODULE_SENSOR_CHANNEL_C, BC_PULSE_COUNTER_EDGE_FALL);

    bc_module_sensor_set_mode(BC_MODULE_SENSOR_CHANNEL_B, BC_MODULE_SENSOR_MODE_INPUT);
    // Pullup is enabled in the task just before measuring

    // Initialize ADC channel - wind direction
    bc_adc_init();
    bc_adc_set_event_handler(BC_ADC_CHANNEL_A5, adc_event_handler, NULL);
    bc_adc_resolution_set(BC_ADC_CHANNEL_A5, BC_ADC_RESOLUTION_12_BIT);

    // Battery module voltage
    bc_module_battery_init();
    bc_module_battery_set_event_handler(battery_event_handler, NULL);
    bc_module_battery_set_update_interval(BATTERY_UPDATE_INTERVAL);

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
    bc_module_climate_set_update_interval_thermometer(MEASURE_INTERVAL_SECONDS * 1000);
    bc_module_climate_set_update_interval_hygrometer(MEASURE_INTERVAL_SECONDS * 1000);
    bc_module_climate_set_update_interval_lux_meter(TRANSMIT_PERIOD_MINUTES * 60 * 1000);
    bc_module_climate_set_update_interval_barometer(TRANSMIT_PERIOD_MINUTES * 60 * 1000);
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
    uint32_t counter = (float)bc_pulse_counter_get(BC_MODULE_SENSOR_CHANNEL_A);
    bc_pulse_counter_reset(BC_MODULE_SENSOR_CHANNEL_A);

    bc_log_debug("wind cnt: %d", counter);

    // Get current wind speed, one pulse per second ~ 2.4kmph
    windSpeed = (((float) counter) / ((float)MEASURE_INTERVAL_SECONDS)) * 0.66666f; // 2.4km/h ~ 0.66666m/s

    // Save maximum wind speed value
    if(windSpeed > windSpeedMaximum)
    {
        windSpeedMaximum = windSpeed;
    }

    // Add value to stream and get average
    bc_data_stream_feed(&stream_wind_speed, &windSpeed);
    bc_data_stream_get_average(&stream_wind_speed, &windSpeedAverage);

    bc_log_debug("speed %f", windSpeed);

    // Read temperature and save it to the buffer stream
    bc_tmp112_measure(&temp);
    bc_data_stream_feed(&stream_internal_temperature, &internalTemperature);
    bc_data_stream_get_average(&stream_internal_temperature, &internalTemperatureAverage);

    // Rain
    uint32_t cnt = bc_pulse_counter_get(BC_MODULE_SENSOR_CHANNEL_C);
    bc_pulse_counter_reset(BC_MODULE_SENSOR_CHANNEL_C);
    bc_log_debug("Rain cnt: %d", cnt);
    if (cnt > 0)
    {
        float cntMM = ((float) cnt) * RAINFALL_PULSE_MM;
        rainTotalMM += cntMM;
        rainMM += cntMM;
    }

    bc_scheduler_plan_current_relative(MEASURE_INTERVAL_SECONDS * 1000);
}
