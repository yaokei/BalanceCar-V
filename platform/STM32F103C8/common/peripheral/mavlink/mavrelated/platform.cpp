
#include "platform.h"

// hardware drivers
#include "stm32f10x.h"
#include "core_cm3.h"
#include "flash.h"
#include "usart1.h"
#include "usart.h"
// tasks
#include "topics.h"
#include "sys_clock_config.h"

#include "library.h"

// initialize hardware
void hal_config(void)
{
   // usart1_config();                               //marverlink
   usart_init();                                    //bluetooth  
   // write(dev_flash, NULL, NULL);//初始化时写入
    task_config();
}

// start tasks and rtos
void task_config(void)
{
    char *argv[] = {"", "start"};
    int argc = sizeof(argv) / sizeof(char*);

   // read(dev_flash, NULL, NULL);
    topics_config();
    extern void mavlink_main(int argc, char *argv[]);
    mavlink_main(argc, argv);
}

void reboot()
{
    NVIC_SystemReset();
}

// return boot time in us
uint32_t time(void)
{
    return (uint32_t)sys_time();
}

// @us: sleep time in us
void usleep(uint32_t us)
{
    uint32_t timestamp = time();
    while (time() < timestamp + us);
}

// write data to file(device)
// @fd    : file handle
// @buf   : write buffer
// @nbytes: data length
int write(int fd, const void *buf, size_t nbytes)
{
    int ret(-1);

    if (fd == dev_uart1) {
        ret = usart1_write((uint8_t*)buf, nbytes);

    } else if (fd == dev_flash) {
        flash_save();
    }

    return ret;
}

// read data from file(device)
// @fd    : file handle
// @buf   : read buffer
// @nbytes: data length
int read(int fd, const void *buf, size_t nbytes)
{
    int ret = -1;

    if (fd == dev_uart1) {
        ret = usart1_read((uint8_t*)buf, nbytes);

    } else if (fd == dev_flash) {
        flash_read();
    }

    return ret;
}


/*
 * schedule
 */

struct schedule_s {
    int priority;
    void (*f)(void*);
    char name[15];
    void *arg;
    schedule_s *next;
};

static struct schedule_s *head(NULL);

int schedule_register(void (*f)(void *), void *arg, int priority)
{
    schedule_s *p = head;
    schedule_s *p_prev(NULL);

    while (p && p->priority < priority) {
        p_prev = p;
        p = p->next;
    }

    schedule_s *s = new (schedule_s);

    if (s) {
        s->next = p;
        s->priority = priority;
        s->f = f;
        s->arg = arg;
        if (p_prev) {
            p_prev->next = s;
        } else {
            head = s;
        }
        return 0;

    } else {
        return -1;
    }
}

uint32_t time_used(0);

void schedule(void)
{
    uint32_t timestamp = time();

    schedule_s *p(NULL);
    for(p = head; p; p = p->next) {
        if (p->priority < 100) {
            (*(p->f))(p->arg);
        }
    }

    time_used = time() - timestamp;
}

void loop(void)
{
  uart_databuf_get();
    schedule_s *p(NULL);
    for (p = head; p; p = p->next) {
        if (p->priority >= 100) {
            (*(p->f))(p->arg);
        }
    }
}
