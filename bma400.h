/**
 * Copyright (C) 2017 - 2018 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * @file       bma400.h
 * @date       25 Sep 2018
 * @version    1.5.0
 * @brief
 *
 */
/*! @file bma400.h */
/*!
 * @defgroup BMA400 SENSOR API
 * @{
 */

#ifndef BMA400_H__
#define BMA400_H__
/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif
/*********************************************************************/
/* header files */

#include "bma400_defs.h"
/*********************************************************************/
/* (extern) variable declarations */
/*********************************************************************/
/* function prototype declarations */
/*!
 * @brief This API is the entry point, Call this API before using other APIs.
 * This API reads the chip-id of the sensor which is the first step to
 * verify the sensor and also it configures the read mechanism of SPI and
 * I2C interface.
 *
 * @param[in,out] dev : Structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bma400_init(struct bma400_dev *dev);

/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 *
 * @param[in] reg_addr : Register address from where the data to be written.
 * @param[in] reg_data : Pointer to data buffer which is to be written
 *                       in the reg_addr of sensor.
 * @param[in] len      : No of bytes of data to write..
 * @param[in] dev      : Structure instance of bma400_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bma400_set_regs(uint8_t reg_addr, uint8_t *reg_data, uint8_t len, const struct bma400_dev *dev);

/*!
 * @brief This API reads the data from the given register address of sensor.
 *
 * @param[in] reg_addr  : Register address from where the data to be read
 * @param[out] reg_data : Pointer to data buffer to store the read data.
 * @param[in] len       : No of bytes of data to be read.
 * @param[in] dev       : Structure instance of bma400_dev.
 *
 * @note For most of the registers auto address increment applies, with the
 * exception of a few special registers, which trap the address. For e.g.,
 * Register address - 0x14(BMA400_FIFO_DATA_ADDR)
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bma400_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint8_t len, const struct bma400_dev *dev);

/*!
 * @brief This API is used to perform soft-reset of the sensor
 * where all the registers are reset to their default values except 0x4B.
 *
 * @param[in] dev       : Structure instance of bma400_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bma400_soft_reset(const struct bma400_dev *dev);

/*!
 * @brief This API is used to set the power mode of the sensor.
 *
 * @param[in] power_mode  : Macro to select power mode of the sensor.
 * @param[in] dev         : Structure instance of bma400_dev.
 *
 * Possible value for power_mode :
 *   - BMA400_NORMAL_MODE
 *   - BMA400_SLEEP_MODE
 *   - BMA400_LOW_POWER_MODE
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bma400_set_power_mode(uint8_t power_mode, const struct bma400_dev *dev);

/*!
 * @brief This API is used to get the power mode of the sensor
 *
 * @param[out] power_mode  : power mode of the sensor.
 * @param[in] dev          : Structure instance of bma400_dev.
 *
 * * Possible value for power_mode :
 *   - BMA400_NORMAL_MODE
 *   - BMA400_SLEEP_MODE
 *   - BMA400_LOW_POWER_MODE
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bma400_get_power_mode(uint8_t *power_mode, const struct bma400_dev *dev);

/*!
 * @brief This API is used to get the accel data along with the sensor-time
 *
 * @param[in] data_sel     : Variable to select sensor data only
 *                           or data along with sensortime
 * @param[in,out] accel    : Structure instance to store data
 * @param[in] dev          : Structure instance of bma400_dev
 *
 * Assignable macros for "data_sel" :
 *   - BMA400_DATA_ONLY
 *   - BMA400_DATA_SENSOR_TIME
 *
 * @note : The Accel data value are in LSB based on the range selected
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bma400_get_accel_data(uint8_t data_sel, struct bma400_sensor_data *accel, const struct bma400_dev *dev);

/*!
 * @brief This API is used to set the sensor settings like sensor
 * configurations and interrupt configurations like
 *    - Accelerometer configurations (Like ODR,OSR,range...)
 *    - Tap configurations
 *    - Activity change configurations
 *    - Gen1/Gen2 configurations
 *    - Orient change configurations
 *    - Step counter configurations
 *
 * @param[in] conf         : Structure instance of the configuration structure
 * @param[in] n_sett       : Number of settings to be set
 * @param[in] dev          : Structure instance of bma400_dev
 *
 * @note : Fill in the value of the required configurations in the conf structure
 * (Examples are mentioned in the readme.md) before calling this API
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bma400_set_sensor_conf(const struct bma400_sensor_conf *conf, uint16_t n_sett, const struct bma400_dev *dev);

/*!
 * @brief This API is used to get the sensor settings like sensor
 * configurations and interrupt configurations and store
 * them in the corresponding structure instance
 *
 * @param[in] conf         : Structure instance of the configuration structure
 * @param[in] n_sett       : Number of settings to be obtained
 * @param[in] dev          : Structure instance of bma400_dev.
 *
 * @note : Call the API and the settings structure will be updated with the
 * sensor settings
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bma400_get_sensor_conf(struct bma400_sensor_conf *conf, uint16_t n_sett, const struct bma400_dev *dev);

/*!
 * @brief This API is used to set the device specific settings like
 *  - BMA400_AUTOWAKEUP_TIMEOUT
 *  - BMA400_AUTOWAKEUP_INT
 *  - BMA400_AUTO_LOW_POWER
 *  - BMA400_INT_PIN_CONF
 *  - BMA400_INT_OVERRUN_CONF
 *  - BMA400_FIFO_CONF
 *
 * @param[in] conf         : Structure instance of the configuration structure.
 * @param[in] n_sett       : Number of settings to be set
 * @param[in] dev          : Structure instance of bma400_dev.
 *
 * @note : Fill in the value of the required configurations in the conf structure
 * (Examples are mentioned in the readme.md) before calling this API
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bma400_set_device_conf(const struct bma400_device_conf *conf, uint8_t n_sett, const struct bma400_dev *dev);

/*!
 * @brief This API is used to get the device specific settings and store
 * them in the corresponding structure instance
 *
 * @param[in] conf         : Structure instance of the configuration structure
 * @param[in] n_sett       : Number of settings to be obtained
 * @param[in] dev          : Structure instance of bma400_dev.
 *
 * @note : Call the API and the settings structure will be updated with the
 * sensor settings
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bma400_get_device_conf(struct bma400_device_conf *conf, uint8_t n_sett, const struct bma400_dev *dev);

/*!
 * @brief This API is used to get the status of all the interrupts
 * whether they are asserted or not
 *
 * @param[in] int_status   : Interrupt status of sensor
 * @param[in] dev          : Structure instance of bma400_dev.
 *
 * @note : Interrupt status of the sensor determines which all
 * interrupts are asserted at any instant of time
 *
 * Interrupt status macros :
 *  - BMA400_WAKEUP_INT_ASSERTED
 *  - BMA400_ORIENT_CH_INT_ASSERTED
 *  - BMA400_GEN1_INT_ASSERTED
 *  - BMA400_GEN2_INT_ASSERTED
 *  - BMA400_FIFO_FULL_INT_ASSERTED
 *  - BMA400_FIFO_WM_INT_ASSERTED
 *  - BMA400_DRDY_INT_ASSERTED
 *  - BMA400_INT_OVERRUN_ASSERTED
 *  - BMA400_STEP_INT_ASSERTED
 *  - BMA400_S_TAP_INT_ASSERTED
 *  - BMA400_D_TAP_INT_ASSERTED
 *  - BMA400_ACT_CH_X_ASSERTED
 *  - BMA400_ACT_CH_Y_ASSERTED
 *  - BMA400_ACT_CH_Z_ASSERTED
 *
 * @note : Call the API and then use the above macros to
 * check whether the interrupt is asserted or not
 *
 * if (int_status & BMA400_FIFO_FULL_INT_ASSERTED) {
 *     printf("\n FIFO FULL INT ASSERTED");
 * }
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bma400_get_interrupt_status(uint16_t *int_status, const struct bma400_dev *dev);

/*!
 * @brief This API is used to get the enable/disable
 * status of the various interrupts
 *
 * @param[in] int_select   : Structure to select specific interrupts
 * @param[in] n_sett       : Number of interrupt settings enabled / disabled
 * @param[in] dev          : Structure instance of bma400_dev.
 *
 * @note : Select the needed interrupt type for which the status of it whether
 * it is enabled/disabled is to be known in the int_select->int_sel, and the
 * output is stored in int_select->conf either as BMA400_ENABLE/BMA400_DISABLE
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bma400_get_interrupts_enabled(struct bma400_int_enable *int_select, uint8_t n_sett, const struct bma400_dev *dev);

/*!
 * @brief This API is used to enable the various interrupts
 *
 * @param[in] int_select   : Structure to enable specific interrupts
 * @param[in] n_sett       : Number of interrupt settings enabled / disabled
 * @param[in] dev          : Structure instance of bma400_dev.
 *
 * @note : Multiple interrupt can be enabled/disabled by
 *    struct interrupt_enable int_select[2];
 *
 *    int_select[0].int_sel = BMA400_FIFO_FULL_INT_EN;
 *    int_select[0].conf = BMA400_ENABLE;
 *
 *    int_select[1].int_sel = BMA400_FIFO_WM_INT_EN;
 *    int_select[1].conf = BMA400_ENABLE;
 *
 *    rslt = bma400_enable_interrupt(&int_select, 2, dev);
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bma400_enable_interrupt(const struct bma400_int_enable *int_select, uint8_t n_sett, const struct bma400_dev *dev);

/*!
 * @brief This API is used to get the step counter output in form
 * of number of steps in the step_count value
 *
 * @param[out] step_count      : Number of step counts
 * @param[out] activity_data   : Activity data WALK/STILL/RUN
 * @param[in] dev              : Structure instance of bma400_dev.
 *
 *  activity_data   |  Status
 * -----------------|------------------
 *  0x00            | BMA400_STILL_ACT
 *  0x01            | BMA400_WALK_ACT
 *  0x02            | BMA400_RUN_ACT
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bma400_get_steps_counted(uint32_t *step_count, uint8_t *activity_data, const struct bma400_dev *dev);

/*!
 * @brief This API is used to get the temperature data output
 *
 * @note Temperature data output must be divided by a factor of 10
 * Consider temperature_data = 195 ,
 * Then the actual temperature is 19.5 degree Celsius
 *
 * @param[in,out] temperature_data   : Temperature data
 * @param[in] dev                    : Structure instance of bma400_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bma400_get_temperature_data(int16_t *temperature_data, const struct bma400_dev *dev);

/*!
 *  @brief This API writes fifo_flush command to command register.
 *  This action clears all data in the FIFO
 *
 *  @param[in] dev           : Structure instance of bma400_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bma400_set_fifo_flush(const struct bma400_dev *dev);

/*!
 * @brief This API reads the FIFO data from the sensor
 *
 * @note User has to allocate the FIFO buffer along with
 * corresponding fifo read length from his side before calling this API
 * as mentioned in the readme.md
 *
 * @note User must specify the number of bytes to read from the FIFO in
 * fifo->length , It will be updated by the number of bytes actually
 * read from FIFO after calling this API
 *
 * @param[in,out] fifo      : Pointer to the fifo structure.
 *
 * @param[in,out] dev       : Structure instance of bma400_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bma400_get_fifo_data(struct bma400_fifo_data *fifo, const struct bma400_dev *dev);

/*!
 * @brief This API parses and extracts the accelerometer frames, FIFO time
 * and control frames from FIFO data read by the "bma400_get_fifo_data" API
 * and stores it in the "accel_data" structure instance.
 *
 * @note The bma400_extract_accel API should be called only after
 * reading the FIFO data by calling the bma400_get_fifo_data() API
 * Please refer the readme.md for usage.
 *
 * @param[in,out] fifo        : Pointer to the fifo structure.
 *
 * @param[out] accel_data     : Structure instance of bma400_sensor_data where
 *                              the accelerometer data from FIFO is extracted
 *                              and stored after calling this API
 *
 * @param[in,out] frame_count : Number of valid accelerometer frames requested
 *                              by user is given as input and it is updated by
 *                              the actual frames parsed from the FIFO
 *
 * @param[in] dev             : Structure instance of bma400_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bma400_extract_accel(struct bma400_fifo_data *fifo,
			    struct bma400_sensor_data *accel_data,
			    uint16_t *frame_count,
			    const struct bma400_dev *dev);

/*!
 * @brief This is used to perform self test of accelerometer in BMA400
 *
 * @param[in] dev    : Structure instance of bma400_dev.
 *
 * @note The return value of this API gives us the result of self test.
 *
 * @note Performing self test does soft reset of the sensor, User can
 * set the desired settings after performing the self test.
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error / +ve value -> Self-test fail
 *
 *   Return value                  |   Result of self test
 * --------------------------------|---------------------------------
 *  BMA400_OK                      |  Self test success
 *  BMA400_W_SELF_TEST_FAIL        |  self test fail
 */
int8_t bma400_perform_self_test(const struct bma400_dev *dev);

/*!
 * @brief This API is used to set the step counter's configuration
 * parameters from the registers 0x59 to 0x71
 */
int8_t bma400_set_step_counter_param(uint8_t *sccr_conf, const struct bma400_dev *dev);

#ifdef __cplusplus
}
#endif  /* End of CPP guard */

#endif  /* BMA400_H__ */
/** @}*/
