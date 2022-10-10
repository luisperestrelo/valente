#include <stdio.h>
#include <linux/i2c-dev.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "pigpio.h"
#include "bmi2.h"
#include "bmi270.h"

#define DPS_TO_RPM 60.0/360.0
#define BMI270_I2C_ADDR 0x69 //The I2C address of the BMI270 is 0x69 becaue the SDO pin is being pulled to 'VDDIO'
                             //If it were being pulled to GND, the I2C address would be 0x68


//Callback functions to interface with the BMI270, these are called by the BMI270 driver*/ddd
/*!
 * @brief    Callback function to interface with the BMI270 API 
 */
int8_t bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
/*!
 * @brief    Callback function to interface with the BMI270 API 
 */
int8_t bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
/*!
 * @brief    Callback function to interface with the BMI270 API 
 */
void bmi2_delay_us(uint32_t period);
/*!
 * @brief    Callback function to interface with the BMI270 API 
 */
void bmi2_intr1_callback(void);

//End of interface functions


//Configuration functions

/*!
 * @brief    Function to configure the BMI270 sensor
 */
int8_t configure_sensor(struct bmi2_dev *dev);

/*!
 * @brief    Initialize the BMI270 and configure the sensor 
 */
int8_t initBMI270();

//End of Configuration functions



/*!
  * @brief    Function to read the sensor data
*/
int8_t read_data(struct bmi2_sens_data *sensor_data);

/*!
 * @brief    Prints both the unfiltered and filtered sensor data
 */
void output_values(float rpm, float filtered_rpm);

/*!
 * @brief    function to convert the raw data from the chosen axis into meaningful values
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);

/*!
 * @brief applies a moving average filter to the data. buffer size is set to 10
 */
float moving_average_filter(float *buffer, float new_value, int buffer_size);


/* global variables */
static struct bmi2_dev bmi2;
uint8_t i2c_bus;



int main(void)
{
    int8_t rslt; //variable that stores error codes. Refer to bmi2_defs.h for error codes
                 // 0, which is defined as BMI2_OK, means no error

    //float x = 0;
    //float y = 0;
    float z = 0;
    float avg_rpm = 0;
    float buffer[10];

    for(int i=0; i<10; i++)
      buffer[i]=0;

    
    rslt = initBMI270(); // initialize the module and configure the sensor
    
    if (rslt == BMI2_OK) // if both init and configure_sensor were OK, begin polling.
    {
      while(1)
      {
        struct bmi2_sens_data sensor_data;
        rslt = read_data(&sensor_data);

        if (rslt == BMI2_OK)
        {
          z = lsb_to_dps(sensor_data.gyr.z, 2000, bmi2.resolution);

          avg_rpm = moving_average_filter(buffer, z*DPS_TO_RPM, 10);

          output_values(z, avg_rpm);

          // gpioDelay(100000); // 100ms
        }

        else 
        {
        printf("BMI270 sensor data read failed. Error code: %d. Refer to bmi2_defs.h. Exiting . . .\n", rslt);
        break;
        }
      }
    }
}

int8_t initBMI270(){


  // config I2C
    int8_t rslt;
    gpioInitialise();

  	i2c_bus = BMI270_I2C_ADDR;
	  bmi2.intf_ptr = &i2c_bus;
    bmi2.intf = BMI2_I2C_INTF;
    bmi2.read = (bmi2_read_fptr_t)bmi2_i2c_read;
    bmi2.write = (bmi2_write_fptr_t)bmi2_i2c_write;
    bmi2.delay_us = (bmi2_delay_fptr_t)bmi2_delay_us;
    bmi2.read_write_len = 32;
    bmi2.config_file_ptr = NULL;

    // End config I2C

    //init and configure module
     rslt = bmi270_init(&bmi2);

    if (rslt == BMI2_OK)
    {
      printf("BMI270 init was successful\n");
    }
    else
    {
      printf("BMI270 init failed. Error code: %d. Refer to bmi2_defs.h\n", rslt);
    }

    
    if (rslt == BMI2_OK)
    {
      //configure sensor
      rslt = configure_sensor(&bmi2);
      if (rslt == BMI2_OK)
      {
        printf("BMI270 sensor configuration was successful\n");
      }
      else
      {
        printf("BMI270 sensor configuration failed. Error code: %d. Refer to bmi2_defs.h\n", rslt);
      }
    }

    return rslt;


}

int8_t read_data(struct bmi2_sens_data *sensor_data){
        int8_t rslt;
        rslt = bmi2_get_sensor_data(sensor_data, &bmi2);
        return rslt;
}

void output_values(float rpm, float filtered_rpm){
          printf("Z RPM = %4.2f\t", rpm*DPS_TO_RPM);
          printf("AVG RPM = %4.2f\n", filtered_rpm);
          gpioDelay(100000); // 100ms
}

int8_t bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
	  uint16_t DevAddress = dev_addr ;

    int8_t rslt = 0; // Return BMI2_OK (= 0) for Success, non-zero for failure. Refer to bmi2_defs.h for error codes

    int i2c_handle = 0;

    i2c_handle = i2cOpen(1, DevAddress, 0);

    if(i2c_handle < 0)
    {
        printf("Error opening I2C device");
        rslt = -1;
    }

    if(rslt == BMI2_OK)
    {
      i2cReadI2CBlockData(i2c_handle, (unsigned int)reg_addr, (char*)reg_data, len);
    }
    i2cClose(i2c_handle);

    return rslt;
}

int8_t bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    int8_t rslt = 0; // Return BMI2_OK (= 0) for Success, non-zero for failure. Refer to bmi2_defs.h for error codes

    int i2c_handle = 0;
	  uint8_t dev_addr = *(uint8_t*)intf_ptr;
	  uint16_t DevAddress = dev_addr << 0;

    i2c_handle = i2cOpen(1, DevAddress, 0);

    if(i2c_handle < 0)
    {
      printf("Error opening I2C device");
      rslt = -1;
    }

    if(rslt == 0)
    {
      char *data_buf = malloc(len + 1);
      data_buf[0] = reg_addr;
      memcpy(&data_buf[1], reg_data, len);
      i2cWriteDevice(i2c_handle, data_buf, len + 1);
      free(data_buf);
    }

    i2cClose(i2c_handle);

    return rslt;
}

float moving_average_filter(float *buffer, float new_value, int buffer_size)
{
    float sum = 0;
    for (int i = 0; i < buffer_size - 1; i++)
    {
      buffer[i] = buffer[i + 1];
      sum += buffer[i];
    }
    buffer[buffer_size - 1] = new_value;
    sum += new_value;
    return sum / buffer_size;
}

void bmi2_delay_us(uint32_t period)
{
    gpioDelay(period);    //micros
}

static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
  float half_scale = ((float)(1 << bit_width) / 2.0f);

  return (dps / ((half_scale) + BMI2_GYR_RANGE_2000)) * (val);
}


/* configures the sensor with the following settings:
    - 2000 dps range
    - 100 Hz filter bandwidth
    - 16 bit resolution
    - various other settings
    - Even though we only use the gyro, the accel is also configured.
*/
int8_t configure_sensor(struct bmi2_dev *dev)
{
  int8_t rslt;
  uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };

  struct bmi2_int_pin_config int_pin_cfg;
  int_pin_cfg.pin_type = BMI2_INT1;
  int_pin_cfg.int_latch = BMI2_INT_NON_LATCH;
  int_pin_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
  int_pin_cfg.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
  int_pin_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
  int_pin_cfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;

  struct bmi2_sens_config sens_cfg[2];
  sens_cfg[0].type = BMI2_ACCEL;
  sens_cfg[0].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
  sens_cfg[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
  sens_cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE,
                      sens_cfg[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
  sens_cfg[1].type = BMI2_GYRO;
  sens_cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
  sens_cfg[1].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
  sens_cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
  sens_cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
  sens_cfg[1].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

  rslt = bmi2_set_int_pin_config(&int_pin_cfg, dev);
  if (rslt != BMI2_OK)
    return rslt;

  rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, dev);
  if (rslt != BMI2_OK)
    return rslt;

  rslt = bmi2_set_sensor_config(sens_cfg, 2, dev);
  if (rslt != BMI2_OK)
    return rslt;

  rslt = bmi2_sensor_enable(sens_list, 2, dev);
  if (rslt != BMI2_OK)
    return rslt;

  return rslt;
}
