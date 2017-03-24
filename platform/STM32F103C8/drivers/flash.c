/************************************************************
 *                         R&A Works
 *
 * @file  :
 * @brief :
 *
 * @author:
 *
 * www.raaworks.com
 ************************************************************/
 
#include "stm32f10x_flash.h" 
#include "stdlib.h"
#include "flash.h"
#include "param.h"

#define PARAM_FLASH_BASE    0x0801FC00                  // start address of the last page
#define PARAM_END_ADDRESS   PARAM_FLASH_BASE + 0x3F0    // end of last page

/**
 * Erase all the page
 */
void flash_erase_page(uint32_t addr)
{
    FLASH_Unlock();
    FLASH_ErasePage(addr);
    //FLASH_Lock();
}

/**
 * Save all the parameters in float for param.c
 *
 * @return status
	 0	one or some of the parameters could not be saved
	 1  all of the parameters are saved
 */
uint8_t flash_save(void)
{
    uint32_t i;
    uint8_t status = 1;
    uint32_t addr = PARAM_END_ADDRESS;
    int32_t val;
    flash_erase_page(PARAM_FLASH_BASE);

    for (i = 0; i < param_count(); i++){
        if (param_get(i,&val) == 0){
            status &= (FLASH_COMPLETE == FLASH_ProgramWord(addr, (uint32_t)val));
            addr -= 4;
        }
    }

    FLASH_Lock();

    return status;
}

/**
 * Read all the parameters in flash for param.c
 *
 * @return status
		0		one or some of the parameters could not found in flash
		1   all of the parameters are read successfully
 */
int8_t flash_read(void)
{
    uint32_t i;
    volatile uint32_t val;
    int8_t status = -1;
    uint32_t addr = PARAM_END_ADDRESS;

    for (i = 0; i < param_count(); i++){
        val = (*(uint32_t *)addr);

        if (val != 0xffffffff){
            status |= param_set(i, (uint32_t*)&val);
        }

        addr -= 4;
    }

    return status;
}
