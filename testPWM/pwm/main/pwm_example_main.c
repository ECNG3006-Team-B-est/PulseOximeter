/* pwm example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
   This is modified from the example in the ESP8266 FreeRTOS SDK 
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "esp8266/gpio_register.h"
#include "esp8266/pin_mux_register.h"

#include "driver/pwm.h"

//GPIO3 is RX, we want to use RX as the PWM output for control signal in measurement mode
//GPIO2 is what we want to use for the buzzer subcircuit
#define PWM_0_OUT_IO_NUM   3
#define PWM_1_OUT_IO_NUM   2

// PWM period 1000us(1Khz), same as depth
#define PWM_PERIOD    (1000)
#define PWM_PERIOD_SLOW    (500)
#define PWM_PERIOD_SUPER_SLOW    (150)


#define PWM_DUTY_CYCLE  (500)
static const char *TAG = "pwm_example";

// pwm pin number
const uint32_t pin_num[1] = {
    PWM_0_OUT_IO_NUM,
};

// duties table, real_duty = duties[x]/PERIOD
uint32_t duties[1] = {
    500,
};

// phase table, delay = (phase[x]/360)*PERIOD
float phase[4] = {
    0, 0, 90.0, -90.0,
};

static void pwm_control_gpio3(void *arg)
{
    const uint32_t pin_num_control_pwm = PWM_0_OUT_IO_NUM;
    for(;;)
    {
        //This will be used for the control signal while in measurement mode
        //1kHz PWM signal using GPIO3
        pwm_init(PWM_PERIOD, duties, 1, pin_num_control_pwm);
        //pwm_set_phases(0);
        pwm_start();
    }
}

static void pwm_jankAlarm_gpio2(void *arg)
{    
    const uint32_t pin_num_alarm_pwm = PWM_1_OUT_IO_NUM;
    for(;;)
    {
        //This will be used for the ALARM signal while in report mode for the buzzer
        //PWM signal using GPIO2
        pwm_init(PWM_PERIOD, duties, 1, pin_num_alarm_pwm);
        //pwm_set_phases(0);
        pwm_start();
        int16_t count = 0;

        while (1) 
        {
            count++;
            if(count%9 == 0)
            {
                vTaskDelay(1000/portTICK_RATE_MS);
            }
            pwm_set_period(PWM_PERIOD);
            pwm_start();
            vTaskDelay(170/portTICK_RATE_MS);
            pwm_stop(0x00);
            pwm_set_period(PWM_PERIOD_SLOW);
            pwm_start();
            vTaskDelay(130/portTICK_RATE_MS);
            pwm_stop(0x00);
            pwm_set_period(PWM_PERIOD_SUPER_SLOW);
            pwm_start();
            vTaskDelay(100/portTICK_RATE_MS);
            pwm_stop(0x00);
        }
    }
}
void app_main()
{
    xTaskCreate(pwm_control_gpio3, "control pwm signal", 2048, NULL, 4, NULL);
    printf("Created control pwm task");
    vTaskDelay(1000/portTICK_RATE_MS);
    xTaskCreate(pwm_jankAlarm_gpio2, "alarm pwm signal", 2048, NULL, 10, NULL);
    printf("Created alarm pwm task");
    vTaskDelay(1000/portTICK_RATE_MS);
    
    int16_t count = 0;

    while (1) {
    count++;
    if(count%9 == 0){
        vTaskDelay(1000/portTICK_RATE_MS);
    }
    }
}

//AD536A is a RMS to DC converter which can help with the pulse. 
//Also, we could have used a precision rectifier (check 2012 notes

//After seeing the demo of unit test in lab, suggested that we can increase the filter capacitors to reduce the ripple. It would increase time to charge up and see change though. 
//We can design in terms of the ripple voltage (the notes can be in 2012)