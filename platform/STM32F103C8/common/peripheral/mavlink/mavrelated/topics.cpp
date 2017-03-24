
#include "topics.h"
#include "string.h"

struct topic_s {
    char *name;
    void *data;
    uint16_t size;
    uint8_t queue;
    uint8_t push;
    uint8_t pop;
    topic_s *next;
};

static struct topic_s *head(NULL);

int publish(char *name, void *data)
{
    topic_s *p;

    for (p = head; p; p = p->next) {
        if (!strcmp(p->name, name)) {
            if (p->queue <= 1) {
                memcpy(p->data, data, p->size);

            } else {
                uint8_t temp = (p->push + 1) % p->queue;

                byte *q = (byte*) p->data;
                memcpy((q + p->size * temp), data, p->size);

                p->push = temp;

                if (temp != p->pop) {
                    // there is free space in buffer

                } else {
                    // drop oldest data
                    p->pop = (p->pop + 1) % p->queue;
                }
            }

            break;
        }
    }

    return p ? 0 : -1;
}

int subscribe(char *name, void *data)
{
    topic_s *p;

    for (p = head; p; p = p->next) {
        if (!strcmp(p->name, name)) {
            if (p->queue <= 1) {
                memcpy(data, p->data, p->size);

            } else {
                uint8_t temp = (p->push - p->pop - 1 + p->queue) % p->queue;

                byte *q = (byte*) p->data;

                if (temp > 0) {
                    // there is new data in buffer
                    memcpy(data, (q + p->size * temp), p->size);
                    p->pop = (p->pop + 1) % p->queue;

                } else {
                    // there is no new data in buffer, copy lastest data
                    memcpy(data, (q + p->size * p->pop), p->size);
                }
            }

            break;
        }
    }

    return p ? 0 : -1;
}

static int topic_register_queue(char *name, unsigned size, uint8_t queue)
{
    if (!queue) {
        return -1;
    }

    topic_s *s = new (topic_s);

    void *data(NULL);

    data = new byte[size * queue];

    if (s && data) {
        s->name = name;
        s->data = data;
        s->size = size;
        s->queue = queue;
        s->push = 0;
        s->pop = queue - 1;
        s->next = NULL;

        if (!head) {
            head = s;
        } else {
            topic_s *p = head;
            for (; p->next; p = p->next);
            p->next = s;
        }

        return 0;
    } else {
        return -1;
    }
}

#define TOPIC_DEFINE(_name)                         topic_register_queue(#_name, sizeof(_name##_s), 1);
#define TOPIC_DEFINE_QUEUE(_name, _queue_length)    topic_register_queue(#_name, sizeof(_name##_s), _queue_length);

// creat topics
int topics_config(void)
{
    int ret(0);

    ret += TOPIC_DEFINE(manual)
    ret += TOPIC_DEFINE(accel)
    ret += TOPIC_DEFINE(gyro)
    ret += TOPIC_DEFINE(mag)
    ret += TOPIC_DEFINE(baro)
    ret += TOPIC_DEFINE(mag_calibration)

    ret += TOPIC_DEFINE(sonar)
    ret += TOPIC_DEFINE(flow)

    ret += TOPIC_DEFINE(gps);

    ret += TOPIC_DEFINE(attitude)
    ret += TOPIC_DEFINE(attitude_setpoint)
    ret += TOPIC_DEFINE(local_position)
    ret += TOPIC_DEFINE(local_position_setpoint)
    ret += TOPIC_DEFINE(rates_setpoint)
    ret += TOPIC_DEFINE(attitude_control_status)

    ret += TOPIC_DEFINE(status)
    ret += TOPIC_DEFINE(control_state)
    ret += TOPIC_DEFINE(control_mode)
    ret += TOPIC_DEFINE(motor_limits)
    ret += TOPIC_DEFINE(actuator)
    ret += TOPIC_DEFINE(actuator_output)

    ret += TOPIC_DEFINE_QUEUE(vehicle_command, 1)

    return ret;
}
