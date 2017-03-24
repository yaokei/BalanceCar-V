
#pragma once

#include "stdint.h" 

#ifdef __cplusplus
extern "C"
{  
#endif
    
extern uint8_t flash_save(void);
extern int8_t flash_read(void);

#ifdef __cplusplus
}
#endif
