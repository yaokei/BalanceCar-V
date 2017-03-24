#ifndef __PARAMETERS_H
#define __PARAMETERS_H

#include "stdint.h"


#ifdef __cplusplus
extern "C" {
#endif


extern void params_load_defaults(void);
extern void parameter_update(void);
extern uint8_t init_flag;
#ifdef __cplusplus
}

#endif
#endif


