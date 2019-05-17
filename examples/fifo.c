#include <stdio.h>
#include "bma400.h"

#define GRAVITY_EARTH  (9.80665f) /* Earth's gravity in m/s^2 */

/* Include 2 additional frames to account for
 * the next frame already being in the FIFO
 * and the sensor time frame
 */
#define N_FRAMES       50

/* 50 Frames result in 50*7, 350 bytes.
 * A few extra for the sensor time frame and
 * in case the next frame is available too.
 */
#define FIFO_SIZE      (357 + BMA400_FIFO_BYTES_OVERREAD)

/* Delay to fill FIFO data At ODR of say 100 Hz,
 * 1 frame gets updated in 1/100 = 0.01s i.e. for
 * 50 frames we need 50 * 0.01 =  0.5 seconds delay
 */
#define WAIT_PERIOD_MS 500

void set_interface(enum bma400_intf intf, struct bma400_dev *dev);
void delay_ms(uint32_t period);
int8_t i2c_reg_write(void *intf_ptr, uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t i2c_reg_read(void *intf_ptr, uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t spi_reg_write(void *intf_ptr, uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t spi_reg_read(void *intf_ptr, uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
void print_rslt(int8_t rslt);
float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width);
float sensor_ticks_to_s(uint32_t sensor_time);

int main(int argc, char const *argv[])
{
    struct bma400_dev bma;
    struct bma400_sensor_data accel_data[N_FRAMES] = { 0 };
    struct bma400_fifo_data fifo_frame;
    struct bma400_device_conf fifo_conf;
    struct bma400_sensor_conf conf;
    int8_t rslt;
    uint16_t i;
    uint8_t fifo_buff[FIFO_SIZE] = { 0 };
    uint16_t accel_frames_req = N_FRAMES;
    float x, y, z, t;
    uint8_t n = 20; /* Read the FIFO 20 times */

    set_interface(BMA400_SPI_INTF, &bma);

    rslt = bma400_init(&bma);
    print_rslt(rslt);

    rslt = bma400_soft_reset(&bma);
    print_rslt(rslt);

    /* Select the type of configuration to be modified */
    conf.type = BMA400_ACCEL;

    /* Get the accelerometer configurations which are set in the sensor */
    rslt = bma400_get_sensor_conf(&conf, 1, &bma);
    print_rslt(rslt);

    /* Modify the desired configurations as per macros
     * available in bma400_defs.h file */
    conf.param.accel.odr = BMA400_ODR_100HZ;
    conf.param.accel.range = BMA400_2G_RANGE;
    conf.param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_1;

    /* Set the desired configurations to the sensor */
    rslt = bma400_set_sensor_conf(&conf, 1, &bma);
    print_rslt(rslt);

    fifo_conf.type = BMA400_FIFO_CONF;

    rslt = bma400_get_device_conf(&fifo_conf, 1, &bma);
    print_rslt(rslt);

    fifo_conf.param.fifo_conf.conf_regs = BMA400_FIFO_X_EN | BMA400_FIFO_Y_EN | BMA400_FIFO_Z_EN | BMA400_FIFO_TIME_EN;
    fifo_conf.param.fifo_conf.conf_status = BMA400_ENABLE;

    rslt = bma400_set_device_conf(&fifo_conf, 1, &bma);
    print_rslt(rslt);

    rslt = bma400_set_power_mode(BMA400_NORMAL_MODE, &bma);
    print_rslt(rslt);

    while ((rslt == BMA400_OK) && n)
    {
        fifo_frame.data = fifo_buff;
        fifo_frame.length = FIFO_SIZE;
        bma.delay_ms(WAIT_PERIOD_MS);

        printf("Requested FIFO length : %d\r\n", fifo_frame.length);

        rslt = bma400_get_fifo_data(&fifo_frame, &bma);
        print_rslt(rslt);

        if (rslt != BMA400_OK)
        {
            printf("FIFO read failed\r\n");
        }

        printf("Available FIFO length : %d \r\n", fifo_frame.length);

        do
        {
            accel_frames_req = N_FRAMES;
            rslt = bma400_extract_accel(&fifo_frame, accel_data, &accel_frames_req, &bma);
            print_rslt(rslt);

            if (rslt != BMA400_OK)
            {
                printf("Accelerometer data extraction failed\r\n");
            }

            if (accel_frames_req)
            {
                printf("Extracted FIFO frames : %d \r\n", accel_frames_req);

                printf("Frame index, Ax[m/s2], Ay[m/s2], Az[m/s2]\r\n");
                for (i = 0; i < accel_frames_req; i++)
                {
                    bma.delay_ms(10); /* Wait for 10ms as ODR is set to 100Hz */

                    /* 12-bit accelerometer at range 2G */
                    x = lsb_to_ms2(accel_data[i].x, 2, 12);
                    y = lsb_to_ms2(accel_data[i].y, 2, 12);
                    z = lsb_to_ms2(accel_data[i].z, 2, 12);

                    printf("%d, %.2f, %.2f, %.2f\r\n", i, x, y, z);
                }
            }
        } while (accel_frames_req);

        if (fifo_frame.fifo_sensor_time)
        {
            t = sensor_ticks_to_s(fifo_frame.fifo_sensor_time);

            printf("FIFO sensor time : %.4fs\r\n", t);
        }

        if (fifo_frame.conf_change)
        {
            printf("FIFO configuration change: 0x%X\r\n", fifo_frame.conf_change);

            if (fifo_frame.conf_change & BMA400_FIFO_CONF0_CHANGE)
            {
                printf("FIFO data source configuration changed \r\n");
            }

            if (fifo_frame.conf_change & BMA400_ACCEL_CONF0_CHANGE)
            {
                printf("Accel filt1_bw configuration changed \r\n");
            }

            if (fifo_frame.conf_change & BMA400_ACCEL_CONF1_CHANGE)
            {
                printf("Accel odr/osr/range configuration changed \r\n");
            }
        }

        n--;
    }

    return 0;
}

void set_interface(enum bma400_intf intf, struct bma400_dev *dev)
{
    switch (intf)
    {
        case BMA400_I2C_INTF:
            dev->intf_ptr = NULL; /* To attach your interface device reference */
            dev->delay_ms = delay_ms;
            dev->dev_id = BMA400_I2C_ADDRESS_SDO_LOW;
            dev->read = i2c_reg_read;
            dev->write = i2c_reg_write;
            dev->intf = BMA400_I2C_INTF;
            break;
        case BMA400_SPI_INTF:
            dev->intf_ptr = NULL; /* To attach your interface device reference */
            dev->dev_id = 0; /* Could be used to identify the chip select line. */
            dev->read = spi_reg_read;
            dev->write = spi_reg_write;
            dev->intf = BMA400_SPI_INTF;
            break;
        default:
            printf("Interface not supported.\r\n");
    }
}

void delay_ms(uint32_t period)
{
    /* Wait for a period amount of ms*/
}

int8_t i2c_reg_write(void *intf_ptr, uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

    /* Write to registers using I2C. Return 0 for a successful execution. */
    return -1;
}

int8_t i2c_reg_read(void *intf_ptr, uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

    /* Read from registers using I2C. Return 0 for a successful execution. */
    return -1;
}

int8_t spi_reg_write(void *intf_ptr, uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

    /* Write to registers using SPI. Return 0 for a successful execution. */
    return -1;
}

int8_t spi_reg_read(void *intf_ptr, uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

    /* Read from registers using SPI. Return 0 for a successful execution. */
    return -1;
}

void print_rslt(int8_t rslt)
{
    switch (rslt)
    {
        case BMA400_OK:

            /* Do nothing */
            break;
        case BMA400_E_NULL_PTR:
            printf("Error [%d] : Null pointer\r\n", rslt);
            break;
        case BMA400_E_COM_FAIL:
            printf("Error [%d] : Communication failure\r\n", rslt);
            break;
        case BMA400_E_DEV_NOT_FOUND:
            printf("Error [%d] : Device not found\r\n", rslt);
            break;
        case BMA400_E_INVALID_CONFIG:
            printf("Error [%d] : Invalid configuration\r\n", rslt);
            break;
        case BMA400_W_SELF_TEST_FAIL:
            printf("Warning [%d] : Self test failed\r\n", rslt);
            break;
        default:
            printf("Error [%d] : Unknown error code\r\n", rslt);
            break;
    }
}

float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width)
{
    float half_scale = (float)(1 << bit_width) / 2.0f;

    return GRAVITY_EARTH * val * g_range / half_scale;
}

float sensor_ticks_to_s(uint32_t sensor_time)
{
    return (float)sensor_time * 0.0000390625f;
}
