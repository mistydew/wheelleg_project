#ifndef SMALL_DRIVER_UART_CONTROL_H
#define SMALL_DRIVER_UART_CONTROL_H

#include "zf_common_headfile.h"

#define SMALL_DRIVER_UART              (UART_4)
#define SMALL_DRIVER_BAUDRATE          (460800)
#define SMALL_DRIVER_TX_PIN            (UART4_TX_P14_1)
#define SMALL_DRIVER_RX_PIN            (UART4_RX_P14_0)
#define SMALL_DRIVER_FRAME_LENGTH      (7u)
#define SMALL_DRIVER_FRAME_HEAD        (0xA5u)
#define SMALL_DRIVER_CMD_DUTY          (0x01u)
#define SMALL_DRIVER_CMD_SPEED         (0x02u)

typedef struct
{
    uint8 send_data_buffer[SMALL_DRIVER_FRAME_LENGTH];
    uint8 receive_data_buffer[SMALL_DRIVER_FRAME_LENGTH];
    uint8 receive_data_count;
    uint8 sum_check_data;
    int16 receive_left_speed_data;
    int16 receive_right_speed_data;
} small_driver_value_struct;

extern small_driver_value_struct small_driver_value;

void small_driver_uart_init(void);
void small_driver_set_duty(small_driver_value_struct *driver, int16 left_duty, int16 right_duty);
void small_driver_get_speed(small_driver_value_struct *driver);
void small_driver_control_callback(small_driver_value_struct *driver);

#endif
