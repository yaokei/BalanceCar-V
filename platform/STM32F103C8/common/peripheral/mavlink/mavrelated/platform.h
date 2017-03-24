
#pragma once

#include "stdlib.h"
#include "stdint.h"

#define IMU_X_REV   1
#define IMU_Y_REV   0
#define IMU_Z_REV   1

#define DIR_READ    (1 << 7)    // set bit7
#define DIR_WRITE   0x00        // clear bit7

// device
enum {
    dev_pwm = 0,
    dev_led1,
    dev_led2,
    dev_uart1,
    dev_uart2,
    dev_uart3,
    dev_volatage,
    dev_motor_voltage,
    dev_ultrasonic,
    dev_flash
};

typedef void (*func)(void *);

#ifdef __cplusplus
extern "C" {
#endif

extern void hal_config(void);
extern void task_config(void);

extern void reboot(void);

extern uint32_t time(void);
extern void usleep(uint32_t);

extern int read(int fd, const void *buf, size_t nbytes);
extern int write(int fd, const void *buf, size_t nbytes);
extern int transfer(const uint8_t *cmd, uint8_t *buf, const uint8_t len);

extern void camera_cmd(uint32_t ms);

extern int schedule_register(func, void *, int);
extern void schedule(void);
extern void loop(void);

#ifdef __cplusplus
}
#endif
