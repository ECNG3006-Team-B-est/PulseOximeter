
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "driver/i2c.h"
#include "driver/gpio.h"

#include "esp8266/gpio_register.h"
#include "esp8266/pin_mux_register.h"

#include "driver/pwm.h"

static const char* TAG = "main";

#define PWM_0_OUT_IO_NUM   3
// set pwm period to 2.32 ms
#define PWM_PERIOD    (2320)

#define SCL_IO_PIN                          2                /*!< gpio number for I2C master clock */
#define SDA_IO_PIN                          0               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM                  I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0                /*!< I2C master do not need buffer */

#define ADS1115_SENSOR_ADDR                 0x48             /*!< slave address for MPU6050 sensor */
#define WRITE_BIT                           I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                            I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                        0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                       0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                             0x0              /*!< I2C ack value */
#define NACK_VAL                            0x1              /*!< I2C nack value */
#define LAST_NACK_VAL                       0x2              /*!< I2C last_nack value */

 /**
  * Define the ads1115 register address:
  */
#define CFG_REG         0x01
#define CONVERSION_REG  0x00

  /**
   * @brief i2c master initialization
   */

// used to keep track of samples taken during measuring
int samples = 0;

// global variables for storing outputs of processing functions
float R;
int SPO2;
int pulse_rate;
bool pwm_on;


//global variables for processing RED PPG signal
int16_t previous_red_reading = 0;
int16_t current_red_reading;
int16_t previous_red_difference = 0;
int16_t current_red_difference;
int16_t red_peaks[200];
int16_t red_troughs[200];
int16_t red_peaks_sample_num[200];
int16_t red_troughs_sample_num[200];
int red_sample_cnt = 0;
int peak_i = 0;    // index for red peak array
int trough_i = 0;   // index for red trough array
int red_ac;
int red_dc;

//global variables for processing IR PPG signal
int16_t previous_infrared_reading = 0;
int16_t current_infrared_reading;
int16_t previous_infrared_difference = 0;
int16_t current_infrared_difference;
int16_t infrared_peaks[100];
int16_t infrared_troughs[100];
int16_t infrared_peaks_sample_num[100];
int16_t infrared_troughs_sample_num[100];
int infrared_sample_cnt = 0;
int peak_j = 0;    // index for infrared trough array
int trough_j = 0;   // index for infrared trough array
int infrared_ac;
int infrared_dc;

// pin number, duty period and phase for pwm
const uint32_t pin_num = PWM_0_OUT_IO_NUM;
uint32_t duties = 1160;
float phase = 0;


static esp_err_t i2c_init()
{
  int i2c_master_port = I2C_MASTER_NUM;
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = SDA_IO_PIN;
  conf.sda_pullup_en = 1;
  conf.scl_io_num = SCL_IO_PIN;
  conf.scl_pullup_en = 1;
  conf.clk_stretch_tick = 500; // 500 ticks, Clock stretch is about 210us, you can make changes according to the actual situation.
  ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
  ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
  return ESP_OK;
}

/**
 * @brief test code to write ads1115
 *
 * 1. send data
 * ___________________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | write data_len byte + ack  | stop |
 * --------|---------------------------|-------------------------|----------------------------|------|
 *
 * @param i2c_num I2C port number
 * @param reg_address slave reg address
 * @param data data to send
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */

 // Page 24 Section 9.5.3 First Paragraph
// ads1115 write function (master write to slave)
static esp_err_t i2c_example_master_ads1115_write(i2c_port_t i2c_num, uint8_t reg_address, uint8_t* data, size_t data_len)
{
  int ret;
  // creating a command link 
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  // populate command link with the following bits : start,slave address,data,stop bit
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ADS1115_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN); 
  i2c_master_write(cmd, data, data_len, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  return ret;
}

/**
 * @brief test code to read ads1115
 *
 * 1. send reg address
 * ______________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | stop |
 * --------|---------------------------|-------------------------|------|
 *
 * 2. read data
 * ___________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | read data_len byte + ack(last nack)  | stop |
 * --------|---------------------------|--------------------------------------|------|
 *
 * @param i2c_num I2C port number
 * @param reg_address slave reg address
 * @param data data to read
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */

 // Page 24 Section 9.5.3 Second Paragraph
 // ads1115 read function ( master reads from slave ) 
static esp_err_t i2c_example_master_ads1115_read(i2c_port_t i2c_num, uint8_t reg_address, uint8_t* data, size_t data_len)
{
  // Writing slave address byte first 
  int ret;
  // create command link
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  // start bit
  i2c_master_start(cmd);
  // writing the slave address byte 
  i2c_master_write_byte(cmd, ADS1115_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
  // writing the address pointer byte which is the register being read from
  i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
  // stop bit
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  if (ret != ESP_OK) {
    return ret;
  }

  // Reading from slave 
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ADS1115_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
  i2c_master_read(cmd, data, data_len, LAST_NACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  return ret;
}

// Start PWM
void generateControlSignal()
{
  pwm_init(PWM_PERIOD, &duties, 1, &pin_num);
  pwm_set_phases(&phase);
  pwm_start();
}

static esp_err_t configI2CRed(i2c_port_t i2c_num)
{
  // writing to the config register, refer to Table 8 , Section 9.6.3, Page 28
  uint16_t cfg_bits;
  /* msb of configuration register
   0b 1 100 001 1 (OS, MUX[2:0], PGA[2:0], MODE) for single shot conversion
   0b 0 100 001 0 for continuous conversion */
  // We use single shot conversion
  cfg_bits = 0xC3E3;
  // lsb of configuration register
  // 0b 111 0 0 0 11 (DR[2:0], COMP_MODE, COMP_POL, COMP_LAT, COMP_QUEUE[1:0])
  // for data rate of 860 samples per second, 
  //cfg_bits[1] = 0xE3;
  // writing configuration bits to the configuration register 
  ESP_ERROR_CHECK(i2c_example_master_ads1115_write(i2c_num, CFG_REG, (uint8_t*) &cfg_bits, 2));
  return ESP_OK;
}

static esp_err_t configI2CInfrared(i2c_port_t i2c_num)
{
  // writing to the config register, refer to Table 8 , Section 9.6.3, Page 28
  uint16_t cfg_infrared_bits;
  /* msb of configuration register
   0b 1 101 001 1 (OS, MUX[2:0], PGA[2:0], MODE) for single shot conversion
   0b 0 101 001 0 for continuous conversion */
  // We use single shot conversion
  cfg_infrared_bits = 0xD3E3;
  // lsb of configuration register
  // 0b 111 0 0 0 11 (DR[2:0], COMP_MODE, COMP_POL, COMP_LAT, COMP_QUEUE[1:0])
  // for data rate of 860 samples per second, 
  //cfg_infrared_bits[1] = 0xE3;
  // Commenting this out because i think you don't have to initialize it twice which was causing the error?
  // writing configuration bits to the configuration register 
  ESP_ERROR_CHECK(i2c_example_master_ads1115_write(i2c_num, CFG_REG, (uint8_t *) &cfg_infrared_bits, 2));
  return ESP_OK;
}

void storeADCRed_task ()
{
  uint8_t sensor_data[2];
  int16_t red_reading;
  int ret;
  int tick = 1;
  ret = configI2CRed(I2C_MASTER_NUM);
  if (ret == ESP_OK)
  {
    ESP_LOGI(TAG, "ADS1115 initialized for Red reading on Channel 0!\n");
  }
  else
  {
    ESP_LOGI(TAG, "Error occured while trying to initialize the ADS1115 for Red reading on Channel 0\n");
    i2c_driver_delete(I2C_MASTER_NUM);
  }

  // this should wait 1 tick approximately 1 ms which would ensure conversion is completed
  while(tick != 0)
  {
    tick--;
  }
    
    memset(sensor_data, 0, 2);
    // extracting msb and storing in sensor_data[0] and lsb in sensor_data [1]
    ret = i2c_example_master_ads1115_read(I2C_MASTER_NUM, CONVERSION_REG, sensor_data, 2);

    if (ret == ESP_OK)
    {
      // converting the reading into adc value from 0 to 2^16 - 1
      red_reading = (int16_t)((sensor_data[0] << 8) | sensor_data[1]);
      current_red_reading = red_reading;
      red_sample_cnt++;
      ESP_LOGI(TAG, "Analog 0 Pin Value : %d \n", red_reading);
    }
    else
    {
      ESP_LOGE(TAG, "No ack, sensor not connected...skip...\n");
    }
    
  
}

void storeADCInfrared_task()
{
  uint8_t sensor_data[2];
  int16_t infrared_reading;
  int ret;
  int tick = 1;

  ret = configI2CInfrared(I2C_MASTER_NUM);
  if (ret == ESP_OK)
  {
    ESP_LOGI(TAG, "ADS1115 initialized for Infrared Reading on Channel 1!\n");
  }
  else
  {
    ESP_LOGI(TAG, "Error occured while trying to initialize the ADS1115 for Infrared reading on Channel 1\n");
    i2c_driver_delete(I2C_MASTER_NUM);
  }
  // this should wait 1 tick approximately 1 ms which would ensure conversion is completed
  while(tick != 0)
  {
    tick--;
  }

    memset(sensor_data, 0, 2);
    // extracting msb and storing in sensor_data[0] and lsb in sensor_data [1]
    ret = i2c_example_master_ads1115_read(I2C_MASTER_NUM, CONVERSION_REG, sensor_data, 2);

    if (ret == ESP_OK)
    {
      // converting the reading into adc value from 0 to 2^16 - 1
      infrared_reading = (int16_t)((sensor_data[0] << 8) | sensor_data[1]);
      current_infrared_reading = infrared_reading;
      infrared_sample_cnt++;
      ESP_LOGI(TAG, "Analog 1 Pin Value : %d \n", infrared_reading);
    }
    else
    {
      ESP_LOGE(TAG, "No ack, sensor not connected...skip...\n");
    }
    
  
}
  
  void determineRedPeak_task()
  {
    
       // If the previous red reading is empty then store the current reading into the previous red reading
        if(previous_red_reading == 0)
        {
            previous_red_reading = current_red_reading;
        }
        // If there was a previous red reading then compute the current difference between the current reading and previous reading
        else 
        {
            current_red_difference = current_red_reading - previous_red_reading;
            // If the previous difference is 0 then store the current difference as the previous difference
            if (previous_red_difference == 0)
            {
                previous_red_difference = current_red_difference;
                previous_red_reading = current_red_reading;
            }
            // If there is a previous difference then determine whether a peak or a trough occurs
            else 
            {
                current_red_difference = current_red_reading - previous_red_reading;
                // If the previous difference was positive, there was a positive gradient. If the current difference is negative, it means 
                // the gradient went from positive to negative indicating that a peak occured.
                // The peak is stored in an array and it's sample number is also stored
                if(previous_red_difference > 0 && current_red_difference < 0) 
                {
                    ESP_LOGI(TAG, "Index %d, Red Peak Value : %d \n", peak_i,previous_red_reading);
                    red_peaks[peak_i] = previous_red_reading;
                    red_peaks_sample_num[peak_i] = red_sample_cnt - 1;
                    peak_i++;
                    
                }
                else if(previous_red_difference < 0 && current_red_difference > 0)
                {
                    ESP_LOGI(TAG, "Index %d, Red Trough Value : %d \n", trough_i, previous_red_reading);
                    red_troughs[trough_i] = previous_red_reading;
                    red_troughs_sample_num[trough_i] = red_sample_cnt - 1;
                    trough_i++;

                }
                    // update previous reading to current reading
                    previous_red_reading = current_red_reading;
                    // update previous difference to the current difference after
                    previous_red_difference = current_red_difference;
            }
        }
    
  }

  void determineInfraredPeak_task()
  {

        if(previous_infrared_reading == 0)
        {
            previous_infrared_reading = current_infrared_reading;
        }
        else 
        {
            current_infrared_difference = current_infrared_reading - previous_infrared_reading;
            if (previous_infrared_difference == 0)
            {
                previous_infrared_difference = current_infrared_difference;
                previous_infrared_reading = current_infrared_reading;
            }
            else 
            {
                current_infrared_difference = current_infrared_reading - previous_infrared_reading;
                if(previous_infrared_difference > 0 && current_infrared_difference < 0) 
                {
                    ESP_LOGI(TAG, "Index %d, Infrared Peak Value : %d \n", peak_j, previous_infrared_reading);
                    infrared_peaks[peak_j] = previous_infrared_reading;
                    infrared_peaks_sample_num[peak_j] = infrared_sample_cnt - 1;
                    peak_j++;
                    
                }
                else if(previous_infrared_difference < 0 && current_infrared_difference > 0)
                {
                    ESP_LOGI(TAG, "Index %d, Infrared Trough Value : %d \n", trough_j, previous_infrared_reading);
                    infrared_troughs[trough_j] = previous_infrared_reading;
                    infrared_troughs_sample_num[trough_j] = infrared_sample_cnt - 1;
                    trough_j++;
                    
                }
                    // update previous reading to current reading
                    previous_infrared_reading = current_infrared_reading;
                    // update previous_infrared_difference to current_infrared_difference for next iteration
                    previous_infrared_difference = current_infrared_difference;
                
            }
        }
  }

  // pass ac and dc by reference 
  void separateACDC (int16_t peaks[], int16_t troughs[], int peaks_size, int troughs_size, int* ac, int* dc)
  {
    int total = 0;
    int peak_avg = 0;
    int trough_avg = 0; 
    
    // determine peaks average value
    for(int cnt = 0; cnt <= peaks_size ; cnt++)
    {
      total = total + peaks[cnt]; 
    }
    peak_avg = total/(peaks_size + 1); 
    ESP_LOGI(TAG, "The average peak is : %d ", peak_avg);

    total = 0;
    // determine troughs average value
    for(int cnt = 0; cnt <= troughs_size ; cnt++)
    {
      total = total + troughs[cnt]; 
    }
    trough_avg = total/(troughs_size + 1); 
    ESP_LOGI(TAG, "The average trough is : %d ", trough_avg);
    // determine peak to peak value which is used as AC component
    *ac = peak_avg - trough_avg;
    *dc = (peak_avg + trough_avg)/2;

    ESP_LOGI(TAG, "The AC value is : %d ", *ac);
    ESP_LOGI(TAG, "The DC value is : %d ", *dc);

  }

  void calculateR (int red_ac, int red_dc, int infrared_ac, int infrared_dc)
  {
    R = ((float)red_ac/(float)red_dc) *  ((float)infrared_dc/(float)infrared_ac);
  }
  
  // using this formula to determine SPO2 given by multiple research papers as a starting point
  void determineSPO2(float R)
  {
    SPO2 = 115 - (25*R);
  }
  
  // Will use the infrared ppg sample numbers to determine pulse rate
  void determinePulseRate(int16_t peaks_sample_number[], int16_t troughs_sample_number[], int peaks_size, int troughs_size )
  { 
    int peak_sample_difference;
    int trough_sample_difference;
    int total_difference = 0;
    int average_difference;
    // determine difference in sample number between peaks
    for(int current = 0; current < peaks_size ; current++)
    { 
      peak_sample_difference = peaks_sample_number[current + 1] - troughs_sample_number[current];
      total_difference = total_difference + peak_sample_difference;
    }

    // determine difference in sample number between troughs
    for(int current = 0; current < troughs_size ; current++)
    { 
      trough_sample_difference = troughs_sample_number[current + 1] - troughs_sample_number[current];
      total_difference = total_difference + trough_sample_difference;
    }
    // determine average difference in sample number between peaks and troughs combined
    average_difference = (total_difference)/(peaks_size + troughs_size + 2);
    ESP_LOGI(TAG,"Average sample number difference : %d ", average_difference);
    // we know sample rate is 1.16 ms since 860 samples per second are used in ADS1115
    // we also added a one ms delay after writing to config register when switching channels
    //so multiply the average difference by 2.16 ms to determine the pulse time.
    // 60 times 1/pulse time gives the pulse rate in bpm
    //After checking the putty output, it takes 46 ms per sample so appropriate adjustment was made below.
    //This is because of the additional processing time added on due to processing peaks and troughs.
    // However, 46 ms is 21.7 Hz sampling freq, the maximum heartbeat signal frequency expected is to be 5Hz.
    // As long as the sampling frequency, 21.7 Hz is greater than 2 times to the maximum expected frequency of the signal being sampled
    // Nyquist sampling theorem is satisfied.
    pulse_rate = 60 * 1000 /(average_difference * 46) ;
  }

/*Main function for measurement mode */
/*Measuring Function - Continuously store measurements and determine peaks*/

  void app_main(void)
  {
    ESP_LOGI(TAG, "Starting up\n ");
    i2c_init();
    ESP_LOGI(TAG, "ADS1115 initialized!\n ");
    while(1)
    {
      if(pwm_on != true)
      {
        pwm_on = true;
        generateControlSignal();
      }
      //Measuring Functions
      //Keep measuring until 500 samples obtained for each, resulting in 1000 total 
      //since sampling time is 1.16 ms plus 1 ms overhead in having to write bytes to cfg register,
      //As long as it is under 4 seconds, requirements will be met.
      if(samples < 500)
      {
        storeADCRed_task();
        storeADCInfrared_task();
        determineRedPeak_task();
        determineInfraredPeak_task();
        samples++;
      }
      else 
      {
      // Only process after we get enough samples
      //Processing Functions
        separateACDC(red_peaks, red_troughs, peak_i, trough_i, &red_ac, &red_dc);
        separateACDC(infrared_peaks, infrared_troughs, peak_j, trough_j, &infrared_ac, &infrared_dc);
        ESP_LOGI(TAG,"RED AC : %d ", red_ac);
        ESP_LOGI(TAG,"RED DC : %d ", red_dc);
        ESP_LOGI(TAG,"INFRARED AC : %d ", infrared_ac);
        ESP_LOGI(TAG,"INFRARED DC : %d ", infrared_dc);
        calculateR(red_ac,red_dc,infrared_ac,infrared_dc);
        determineSPO2(R);
        determinePulseRate(infrared_peaks_sample_num, infrared_troughs_sample_num, peak_j, trough_j);

        ESP_LOGI (TAG,"R : %.2f \n", R);
        ESP_LOGI (TAG,"SPO2 : %d %% \n", SPO2);
        ESP_LOGI (TAG,"Pulse Rate : %d bpm \n", pulse_rate);


      // reset all trackers/indexes back to 0 after pulse rate and spo2 is computed
        red_sample_cnt = 0;
        infrared_sample_cnt = 0;
        peak_i = 0 ;
        trough_i = 0;
        peak_j = 0 ;
        trough_j = 0;
        samples = 0;
      // Don't have reporting mode as yet so using 5 seconds to delay which is the time it stays in reporting mode
        vTaskDelay(5000/portTICK_RATE_MS);
      }
    }

  }
