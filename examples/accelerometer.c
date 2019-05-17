#include <stdio.h>
#include "bma400.h"

#define GRAVITY_EARTH (9.80665f) /* Earth's gravity in m/s^2 */

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
    struct bma400_sensor_conf conf;
    struct bma400_sensor_data data;
    int8_t rslt;
    uint8_t n_samples = 200;
    float t, x, y, z;

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

    rslt = bma400_set_power_mode(BMA400_LOW_POWER_MODE, &bma);
    print_rslt(rslt);

    printf("t[s], Ax[m/s2], Ay[m/s2], Az[m/s2]\r\n");

    while (n_samples && (rslt == BMA400_OK))
    {
        bma.delay_ms(10); /* Wait for 10ms as ODR is set to 100Hz */

        rslt = bma400_get_accel_data(BMA400_DATA_SENSOR_TIME, &data, &bma);
        print_rslt(rslt);

        /* 12-bit accelerometer at range 2G */
        x = lsb_to_ms2(data.x, 2, 12);
        y = lsb_to_ms2(data.y, 2, 12);
        z = lsb_to_ms2(data.z, 2, 12);
        t = sensor_ticks_to_s(data.sensortime);

        printf("%.4f, %.2f, %.2f, %.2f\r\n", t, x, y, z);
        n_samples--;
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
