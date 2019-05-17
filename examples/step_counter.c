#include <stdio.h>
#include "bma400.h"

void set_interface(enum bma400_intf intf, struct bma400_dev *dev);
void delay_ms(uint32_t period);
int8_t i2c_reg_write(void *intf_ptr, uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t i2c_reg_read(void *intf_ptr, uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t spi_reg_write(void *intf_ptr, uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t spi_reg_read(void *intf_ptr, uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
void print_rslt(int8_t rslt);

int main(int argc, char const *argv[])
{
    struct bma400_dev bma;
    struct bma400_int_enable step_int;
    int8_t rslt;
    uint8_t test_dur = 30;
    uint32_t step_count;
    uint8_t activity;

    set_interface(BMA400_SPI_INTF, &bma);

    rslt = bma400_init(&bma);
    print_rslt(rslt);

    rslt = bma400_soft_reset(&bma);
    print_rslt(rslt);

    step_int.type = BMA400_STEP_COUNTER_INT_EN;
    step_int.conf = BMA400_ENABLE;

    rslt = bma400_enable_interrupt(&step_int, 1, &bma);
    print_rslt(rslt);

    rslt = bma400_set_power_mode(BMA400_NORMAL_MODE, &bma);
    print_rslt(rslt);

    printf("Steps counted, Activity classifier\r\n");

    while (test_dur)
    {
        bma.delay_ms(1000);

        rslt = bma400_get_steps_counted(&step_count, &activity, &bma);
        printf("%ld", step_count);

        switch (activity)
        {
            case BMA400_STILL_ACT:
                printf(", Still\r\n");
                break;
            case BMA400_WALK_ACT:
                printf(", Walking\r\n");
                break;
            case BMA400_RUN_ACT:
                printf(", Running\r\n");
                break;
            default:
                printf(", undefined\r\n");
                break;
        }

        test_dur--;
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
