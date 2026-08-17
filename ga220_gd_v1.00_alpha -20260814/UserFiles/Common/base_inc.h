#ifndef __BASE_INC_H__
#define __BASE_INC_H__
/*complier library*/
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*option*/
#include "Common/opt_version.h"
#include "Common/opt_cmd.h"
#include "Common/opt_module.h"
/*cpu include*/
#if defined (STM32H743xx)||(STM32H7A3xx)
#include "stm32h7xx_hal.h"
#endif

#if defined (STM32F405xx)||(STM32F427xx)||(STM32F407xx)
#include "stm32f4xx_hal.h"
#endif
#include "Bsp/bsp_gpio.h"

#endif

