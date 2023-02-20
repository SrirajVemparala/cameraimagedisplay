/******************************************************************************
* @file: main.h
*
* @brief: This file consists of function declarations for start of the code
*
* @author: Raghu Sai Phani Sriraj Vemparala
* @references: http://www.ece.ualberta.ca/~elliott/ee552/studentAppNotes/2003_w/misc/bmp_file_format/bmp_file_format.htm
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
Header files
*******************************************************************************/
#include "stm32f4xx_hal.h"

/*******************************************************************************
 Function declarations
*******************************************************************************/
void Error_Handler(void);


/*******************************************************************************
 Macros
*******************************************************************************/
/* Private defines -----------------------------------------------------------*/
#define SD_CS_Pin GPIO_PIN_1
#define SD_CS_GPIO_Port GPIOC


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
