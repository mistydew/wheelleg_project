#include "small_driver_uart_control.h"

small_driver_value_struct small_driver_value;

static uint8 SmallDriver_CheckSum(const uint8 *buffer)
{
    uint8 i;
    uint8 sum = 0;

    for(i = 0; i < (SMALL_DRIVER_FRAME_LENGTH - 1u); i++)
    {
        sum += buffer[i];
    }

    return sum;
}

static void SmallDriver_ResetReceive(small_driver_value_struct *driver)
{
    driver->receive_data_count = 0;
    memset(driver->receive_data_buffer, 0, SMALL_DRIVER_FRAME_LENGTH);
}

static void SmallDriver_InitValue(small_driver_value_struct *driver)
{
    memset(driver->send_data_buffer, 0, SMALL_DRIVER_FRAME_LENGTH);
    memset(driver->receive_data_buffer, 0, SMALL_DRIVER_FRAME_LENGTH);
    driver->receive_data_count = 0;
    driver->sum_check_data = 0;
    driver->receive_left_speed_data = 0;
    driver->receive_right_speed_data = 0;
}

void small_driver_uart_init(void)
{
    SmallDriver_InitValue(&small_driver_value);
    uart_init(SMALL_DRIVER_UART, SMALL_DRIVER_BAUDRATE, SMALL_DRIVER_TX_PIN, SMALL_DRIVER_RX_PIN);
    uart_rx_interrupt(SMALL_DRIVER_UART, 1);
    small_driver_set_duty(&small_driver_value, 0, 0);
    small_driver_get_speed(&small_driver_value);
}

void small_driver_set_duty(small_driver_value_struct *driver, int16 left_duty, int16 right_duty)
{
    driver->send_data_buffer[0] = SMALL_DRIVER_FRAME_HEAD;
    driver->send_data_buffer[1] = SMALL_DRIVER_CMD_DUTY;
    driver->send_data_buffer[2] = (uint8)(((uint16)left_duty >> 8) & 0x00FFu);
    driver->send_data_buffer[3] = (uint8)((uint16)left_duty & 0x00FFu);
    driver->send_data_buffer[4] = (uint8)(((uint16)right_duty >> 8) & 0x00FFu);
    driver->send_data_buffer[5] = (uint8)((uint16)right_duty & 0x00FFu);
    driver->send_data_buffer[6] = SmallDriver_CheckSum(driver->send_data_buffer);

    uart_write_buffer(SMALL_DRIVER_UART, driver->send_data_buffer, SMALL_DRIVER_FRAME_LENGTH);
}

void small_driver_get_speed(small_driver_value_struct *driver)
{
    driver->send_data_buffer[0] = SMALL_DRIVER_FRAME_HEAD;
    driver->send_data_buffer[1] = SMALL_DRIVER_CMD_SPEED;
    driver->send_data_buffer[2] = 0x00u;
    driver->send_data_buffer[3] = 0x00u;
    driver->send_data_buffer[4] = 0x00u;
    driver->send_data_buffer[5] = 0x00u;
    driver->send_data_buffer[6] = SmallDriver_CheckSum(driver->send_data_buffer);

    uart_write_buffer(SMALL_DRIVER_UART, driver->send_data_buffer, SMALL_DRIVER_FRAME_LENGTH);
}

void small_driver_control_callback(small_driver_value_struct *driver)
{
    uint8 receive_data;

    if(uart_query_byte(SMALL_DRIVER_UART, &receive_data))
    {
        if((SMALL_DRIVER_FRAME_HEAD == receive_data) &&
           (SMALL_DRIVER_FRAME_HEAD != driver->receive_data_buffer[0]))
        {
            driver->receive_data_count = 0;
        }

        driver->receive_data_buffer[driver->receive_data_count] = receive_data;
        driver->receive_data_count++;

        if(driver->receive_data_count >= SMALL_DRIVER_FRAME_LENGTH)
        {
            if((SMALL_DRIVER_FRAME_HEAD == driver->receive_data_buffer[0]) &&
               (SmallDriver_CheckSum(driver->receive_data_buffer) == driver->receive_data_buffer[6]))
            {
                if(SMALL_DRIVER_CMD_SPEED == driver->receive_data_buffer[1])
                {
                    driver->receive_left_speed_data =
                        (int16)(((uint16)driver->receive_data_buffer[2] << 8) |
                                (uint16)driver->receive_data_buffer[3]);
                    driver->receive_right_speed_data =
                        (int16)(((uint16)driver->receive_data_buffer[4] << 8) |
                                (uint16)driver->receive_data_buffer[5]);
                }
            }

            SmallDriver_ResetReceive(driver);
        }
    }
}
