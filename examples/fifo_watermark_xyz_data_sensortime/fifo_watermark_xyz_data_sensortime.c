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

/* 39.0625us per tick */
#define SENSOR_TICK_TO_S        (0.0000390625f)

/*! Setting a watermark level in FIFO */
#define FIFO_WATERMARK_LEVEL    UINT16_C(650)

/* Total FIFO buffer size */
#define FIFO_FULL_SIZE          UINT16_C(1024)

/* Add extra bytes to get complete fifo data */
#define FIFO_SIZE               (FIFO_FULL_SIZE + BMA400_FIFO_BYTES_OVERREAD)

/*! Number of accel frames to be extracted from FIFO
 * Calculation:
 * fifo_watermark_level = 650, accel_frame_len = 6, header_byte = 1.
 * fifo_accel_frame_count = (650 / (6 + 1)) = 93 frames
 * NOTE: Extra frames are read in order to get sensor time
 */
#define FIFO_ACCEL_FRAME_COUNT  UINT8_C(150)

/************************************************************************/
/*********                      Main Function               *************/
/************************************************************************/

int main(int argc, char const *argv[])
{
    int8_t rslt;

    uint16_t index;

    struct bma400_fifo_sensor_data accel_data[FIFO_ACCEL_FRAME_COUNT] = { { 0 } };
    struct bma400_int_enable int_en;
    struct bma400_fifo_data fifo_frame;
    struct bma400_device_conf fifo_conf;
    struct bma400_sensor_conf conf;
    uint16_t int_status = 0;
    uint8_t fifo_buff[FIFO_SIZE] = { 0 };
    uint16_t accel_frames_req = FIFO_ACCEL_FRAME_COUNT;
    uint8_t try = 1;

    struct bma400_dev bma;

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

    fifo_conf.type = BMA400_FIFO_CONF;

    rslt = bma400_get_device_conf(&fifo_conf, 1, &bma);
    bma400_check_rslt("bma400_get_device_conf", rslt);

    rslt = bma400_set_power_mode(BMA400_MODE_NORMAL, &bma);
    bma400_check_rslt("bma400_set_power_mode", rslt);

    fifo_conf.param.fifo_conf.conf_regs = BMA400_FIFO_X_EN | BMA400_FIFO_Y_EN | BMA400_FIFO_Z_EN | BMA400_FIFO_TIME_EN;
    fifo_conf.param.fifo_conf.conf_status = BMA400_ENABLE;
    fifo_conf.param.fifo_conf.fifo_watermark = FIFO_WATERMARK_LEVEL;
    fifo_conf.param.fifo_conf.fifo_wm_channel = BMA400_INT_CHANNEL_2;

    rslt = bma400_set_device_conf(&fifo_conf, 1, &bma);
    bma400_check_rslt("bma400_set_device_conf", rslt);

    int_en.type = BMA400_FIFO_WM_INT_EN;
    int_en.conf = BMA400_ENABLE;

    rslt = bma400_enable_interrupt(&int_en, 1, &bma);
    bma400_check_rslt("bma400_enable_interrupt", rslt);

    printf("Read FIFO watermark interrupt XYZ data with sensortime\n");

    while (try <= 10)
    {
        rslt = bma400_get_interrupt_status(&int_status, &bma);
        bma400_check_rslt("bma400_get_interrupt_status", rslt);

        if (int_status & BMA400_ASSERTED_FIFO_WM_INT)
        {
            printf("\n\nIteration : %d\n\n", try);

            fifo_frame.data = fifo_buff;
            fifo_frame.length = FIFO_SIZE;

            rslt = bma400_get_device_conf(&fifo_conf, 1, &bma);
            bma400_check_rslt("bma400_set_device_conf", rslt);

            printf("Watermark level : %d\n", fifo_conf.param.fifo_conf.fifo_watermark);

            printf("Requested FIFO length : %d\n", fifo_frame.length);

            rslt = bma400_get_fifo_data(&fifo_frame, &bma);
            bma400_check_rslt("bma400_get_fifo_data", rslt);

            printf("Available FIFO length : %d\n", fifo_frame.length);

            accel_frames_req = FIFO_ACCEL_FRAME_COUNT;
            printf("Requested FIFO frames : %d\n", accel_frames_req);

            rslt = bma400_extract_accel(&fifo_frame, accel_data, &accel_frames_req, &bma);
            bma400_check_rslt("bma400_extract_accel", rslt);

            if (accel_frames_req)
            {
                printf("Extracted FIFO frames : %d\n", accel_frames_req);

                for (index = 0; index < accel_frames_req; index++)
                {
                    /* 12-bit accelerometer at range 2G */
                    printf("Accel[%d] Raw_X : %d     Raw_Y : %d     Raw_Z : %d \n",
                           index,
                           accel_data[index].x,
                           accel_data[index].y,
                           accel_data[index].z);
                }
            }

            if (fifo_frame.fifo_sensor_time)
            {
                printf("FIFO sensor time : %.4fs\n", (float)(fifo_frame.fifo_sensor_time) * SENSOR_TICK_TO_S);
            }

            if (fifo_frame.conf_change)
            {
                printf("FIFO configuration change: 0x%X\n", fifo_frame.conf_change);

                if (fifo_frame.conf_change & BMA400_FIFO_CONF0_CHANGE)
                {
                    printf("FIFO data source configuration changed\n");
                }

                if (fifo_frame.conf_change & BMA400_ACCEL_CONF0_CHANGE)
                {
                    printf("Accel filt1_bw configuration changed\n");
                }

                if (fifo_frame.conf_change & BMA400_ACCEL_CONF1_CHANGE)
                {
                    printf("Accel odr/osr/range configuration changed\n");
                }
            }

            try++;
        }
    }

    bma400_coines_deinit();

    return rslt;
}
