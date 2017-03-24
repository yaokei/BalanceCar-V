#ifndef __RGB_LED_H
#define __RGB_LED_H


#include "library.h"


#ifdef __cplusplus
 extern "C" {
#endif
extern void rgb_led_init(void);
extern void red_set(void);
extern void green_set(void);
extern void blue_set(void);
extern void rgb_test(uint8_t run);


#ifdef __cplusplus
}
#endif 
#endif

