/*
 * Copyright (C) 2020 Bosch Sensortec GmbH
 *
 * The license is available at root folder
 *
 */

#include <stdio.h>
#include "bma400.h"
#include "common.h"

/************************************************************************/
/*********                      Macros                      *************/
/************************************************************************/

/* Earth's gravity in m/s^2 */
#define GRAVITY_EARTH     (9.80665f)

/* 39.0625us per tick */
#define SENSOR_TICK_TO_S  (0.0000390625f)

/************************************************************************/
/*********                      Static APIs                 *************/
/************************************************************************/

/*!
 *  @brief This internal API converts raw sensor values(LSB) to meters per seconds square.
 *
 *  @param[in] accel_data    : Raw sensor value.
 *  @param[in] g_range       : Accel Range selected (2G, 4G, 8G, 16G).
 *  @param[in] bit_width     : Resolution of the sensor.
 *
 *  @return Accel values in meters per second square.
 *
 */
static float lsb_to_ms2(int16_t accel_data, uint8_t g_range, uint8_t bit_width)
{
    float accel_ms2;
    int16_t half_scale;

    half_scale = 1 << (bit_width - 1);
    accel_ms2 = (GRAVITY_EARTH * accel_data * g_range) / half_scale;

    return accel_ms2;
}

/************************************************************************/
/*********                      Main Function               *************/
/************************************************************************/

int main(int argc, char const *argv[])
{
    struct bma400_dev bma;
    struct bma400_sensor_conf conf;
    struct bma400_sensor_data data;
    struct bma400_int_enable int_en;
    int8_t rslt;
    uint8_t n_samples = 50;
    float t, x, y, z;
    uint16_t int_status = 0;

    /* Interface reference is given as a parameter
     *         For I2C : BMA400_I2C_INTF
     *         For SPI : BMA400_SPI_INTF
     */
    rslt = bma400_interface_init(&bma, BMA400_I2C_INTF);
    bma400_check_rslt("bma400_interface_init", rslt);

    rslt = bma400_init(&bma);
    bma400_check_rslt("bma400_init", rslt);

    rslt = bma400_soft_reset(&bma);
    bma400_check_rslt("bma400_soft_reset", rslt);

    /* Select the type of configuration to be modified */
    conf.type = BMA400_ACCEL;

    /* Get the accelerometer configurations which are set in the sensor */
    rslt = bma400_get_sensor_conf(&conf, 1, &bma);
    bma400_check_rslt("bma400_get_sensor_conf", rslt);

    /* Modify the desired configurations as per macros
     * available in bma400_defs.h file */
    conf.param.accel.odr = BMA400_ODR_100HZ;
    conf.param.accel.range = BMA400_RANGE_2G;
    conf.param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_1;

    /* Set the desired configurations to the sensor */
    rslt = bma400_set_sensor_conf(&conf, 1, &bma);
    bma400_check_rslt("bma400_set_sensor_conf", rslt);

    rslt = bma400_set_power_mode(BMA400_MODE_NORMAL, &bma);
    bma400_check_rslt("bma400_set_power_mode", rslt);

    int_en.type = BMA400_DRDY_INT_EN;
    int_en.conf = BMA400_ENABLE;

    rslt = bma400_enable_interrupt(&int_en, 1, &bma);
    bma400_check_rslt("bma400_enable_interrupt", rslt);

    printf("\nGet accel data - BMA400_DATA_SENSOR_TIME\n");

    printf("Accel Gravity data in m/s^2\n");

    while (n_samples && (rslt == BMA400_OK))
    {
        rslt = bma400_get_interrupt_status(&int_status, &bma);
        bma400_check_rslt("bma400_get_interrupt_status", rslt);

        if (int_status & BMA400_ASSERTED_DRDY_INT)
        {
            rslt = bma400_get_accel_data(BMA400_DATA_SENSOR_TIME, &data, &bma);
            bma400_check_rslt("bma400_get_accel_data", rslt);

            /* 12-bit accelerometer at range 2G */
            x = lsb_to_ms2(data.x, 2, 12);
            y = lsb_to_ms2(data.y, 2, 12);
            z = lsb_to_ms2(data.z, 2, 12);
            t = (float)data.sensortime * SENSOR_TICK_TO_S;

            printf("Acc_Raw_X : %d   Acc_Raw_Y : %d   Acc_Raw_Z : %d", data.x, data.y, data.z);
            printf("\tAcc_ms2_X : %.2f,   Acc_ms2_Y : %.2f,  Acc_ms2_Z :  %.2f,   t(s) : %.4f\n", x, y, z, t);
            n_samples--;
        }
    }

    n_samples = 50;

    printf("\nGet accel data - BMA400_DATA_ONLY\n");

    printf("Accel Gravity data in m/s^2\n");

    while (n_samples && (rslt == BMA400_OK))
    {
        rslt = bma400_get_interrupt_status(&int_status, &bma);
        bma400_check_rslt("bma400_get_interrupt_status", rslt);

        if (int_status & BMA400_ASSERTED_DRDY_INT)
        {
            rslt = bma400_get_accel_data(BMA400_DATA_ONLY, &data, &bma);
            bma400_check_rslt("bma400_get_accel_data", rslt);

            /* 12-bit accelerometer at range 2G */
            x = lsb_to_ms2(data.x, 2, 12);
            y = lsb_to_ms2(data.y, 2, 12);
            z = lsb_to_ms2(data.z, 2, 12);

            printf("Acc_Raw_X : %d   Acc_Raw_Y : %d   Acc_Raw_Z : %d", data.x, data.y, data.z);
            printf("\tAcc_ms2_X : %.2f,   Acc_ms2_Y : %.2f,  Acc_ms2_Z :  %.2f\n", x, y, z);
            n_samples--;
        }
    }

    bma400_coines_deinit();

    return rslt;
}
