/******************************************************************************
* @file: ili9341.h
*
* @brief: This files consists of function decalarations for transferring data to ILI9341
*
* @author: Raghu Sai Phani Sriraj Vemparala
* @references: ILI9341 data sheet
* ILI9341Init(void) reference: https://community.seacomp.com/post/ilitek-ili9341-initialization-code-606242de05d70e2771af3d38
*******************************************************************************/
#ifndef INC_ILI9341_H_
#define INC_ILI9341_H_

/*******************************************************************************
Header files
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 Macros
*******************************************************************************/
#define ILI9341_RST_PORT           	GPIOC
#define ILI9341_RST_PIN             GPIO_PIN_4
#define ILI9341_DC_PORT             GPIOC
#define ILI9341_DC_PIN              GPIO_PIN_5
#define ILI9341_CS_PORT             GPIOA
#define ILI9341_CS_PIN              GPIO_PIN_4
#define ILI9341_LCD_WIDTH           240U
#define ILI9341_LCD_HEIGHT          320U

typedef uint16_t colour_t;


/*******************************************************************************
 Function declarations
*******************************************************************************/
void setaddress(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd);
void WriteCommand(uint8_t command);
void WriteData(uint8_t data);
void ili9341_start();
void ILI9341Reset(void);
void ILI9341Init(void);
void ILI9341_write_Pixel(uint16_t x, uint16_t y, colour_t colour);


#endif
