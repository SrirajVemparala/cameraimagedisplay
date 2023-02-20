/******************************************************************************
* @file: ili9341.c
*
* @brief: This files consists of functions for transferring data to ILI9341
*
* @author: Raghu Sai Phani Sriraj Vemparala
* @references: ILI9341 data sheet
* ILI9341Init(void) reference: https://community.seacomp.com/post/ilitek-ili9341-initialization-code-606242de05d70e2771af3d38
*******************************************************************************/

/*******************************************************************************
Header files
*******************************************************************************/
#include <stddef.h>
#include "stm32f4xx_hal.h"
#include "ili9341.h"

/*******************************************************************************
 Macros
*******************************************************************************/
#define DMA_BUFFER_SIZE 			64U

/*******************************************************************************
 Global variables
*******************************************************************************/
extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_tx;

static volatile bool txComplete;


/*******************************************************************************
* @Function Name: ili9341_start()
* @Description:Start Initialization ILI9341
* @input param : void
* @return: void
*******************************************************************************/
void ili9341_start()
{
	ILI9341Reset();
	ILI9341Init();
}
/*******************************************************************************
* @Function Name: WriteCommand()
* @Description: writes commands to the ILI9341
* @input param : uint8_t
* @return: void
*******************************************************************************/
void WriteCommand(uint8_t command)
{
	HAL_GPIO_WritePin(ILI9341_DC_PORT, ILI9341_DC_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &command, 1U, 100U);
}
/*******************************************************************************
* @Function Name: WriteData()
* @Description: writes data to the ILI9341
* @input param : uint8_t
* @return: void
*******************************************************************************/
void WriteData(uint8_t data)
{
	HAL_GPIO_WritePin(ILI9341_DC_PORT, ILI9341_DC_PIN, GPIO_PIN_SET);
	HAL_SPI_Transmit(&hspi1, &data, 1U, 100U);
}

/*******************************************************************************
* @Function Name: ILI9341Reset()
* @Description: Resets ILI9341 LCD
* @input param : void
* @return: void
*******************************************************************************/
void ILI9341Reset(void)
{
	HAL_GPIO_WritePin(ILI9341_RST_PORT, ILI9341_RST_PIN, GPIO_PIN_RESET);
	HAL_Delay(200UL);
	HAL_GPIO_WritePin(ILI9341_RST_PORT, ILI9341_RST_PIN, GPIO_PIN_SET);
	HAL_Delay(200UL);
}

/*******************************************************************************
* @Function Name: ILI9341Init()
* @Description: Initializes ILI9341
* @input param : void
* @return: void
*******************************************************************************/
void ILI9341Init(void)
{
	HAL_GPIO_WritePin(ILI9341_CS_PORT, ILI9341_CS_PIN, GPIO_PIN_RESET);

	//Software Reset command
	WriteCommand(0x01U);
	HAL_Delay(1000UL);
	//Power control command
	WriteCommand(0xCBU);
	WriteData(0x39U);//Argument 1
	WriteData(0x2CU);//Argument 2
	WriteData(0x00U);//Argument 3
	WriteData(0x34U);//Argument 4_Set Register VD
	WriteData(0x02U);//Argument 5_Set register DDVDH
	//Power Control B
	WriteCommand(0xCFU);
	WriteData(0x00U);//Arg 1
	WriteData(0xC1U);//Arg 2
	WriteData(0x30U);//Arg 3
	//Drive Timing control
	WriteCommand(0xE8U);
	WriteData(0x85U);//Arg 1
	WriteData(0x00U);//Arg 2
	WriteData(0x78U);//Arg 3
	//Driver timing control B
	WriteCommand(0xEAU);
	WriteData(0x00U);//Arg 1
	WriteData(0x00U);//Arg 2
	//Power ON sequence
	WriteCommand(0xEDU);
	WriteData(0x64U);//Arg 1
	WriteData(0x03U);//Arg 2
	WriteData(0x12U);//Arg 3
	WriteData(0x81U);//Arg 4
	//Pump Ratio control
	WriteCommand(0xF7U);
	WriteData(0x20U);//DDVDH - 2*VCI
    //Power control
	WriteCommand(0xC0U);
	WriteData(0x23U);//Set VRH to 0x23 of pump circuit
	//Power control 2
	WriteCommand(0xC1U);
	WriteData(0x10U);
	//Set VCOM
	WriteCommand(0xC5U);
	WriteData(0x3EU);
	WriteData(0x28U);
	//VCOM control command
	WriteCommand(0xC7U);
	WriteData(0x86U);
	//Memory access
	WriteCommand(0x36U);
	WriteData(0x48U);// Column address order and enavle BGR sequence
	//Pixel format
	WriteCommand(0x3AU);
	WriteData(0x55U);//16 bit pixel format- RGB and MCU interface
	//Frame Rate control
	WriteCommand(0xB1U);
	WriteData(0x00U);//focs is considered as clock
	WriteData(0x18U);//frame rate is 63 Hz
	//Display Function control
	WriteCommand(0xB6U);
	WriteData(0x08U);//Interval scan
	WriteData(0x82U);//Scan cycle 5 frames and LCD type normally white
	WriteData(0x27U);//162 Drive lines
	//Gama control enable
	WriteCommand(0xF2U);
	WriteData(0x00U);
	//Sleep out mode
	WriteCommand(0x11U);
	HAL_Delay(120UL);//Delay is mandatory for coming out of sleep to do the next operation
	//Display ON
	WriteCommand(0x29U);
}

/*******************************************************************************
* @Function Name: SetWindow()
* @Description: Sets the start address and end address for adding pixels
* @input param : uint16_t, uint16_t, uint16_t,uint16_t
* @return: void
*******************************************************************************/
void setaddress(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd)
{
	//Set Column address
	WriteCommand(0x2AU);
	WriteData(xStart >> 8);//Starts at 00h
	WriteData(xStart);
	WriteData(xEnd >> 8);
	WriteData(xEnd);//Ends at pixel number 240(Width)
	//Set Row address
	WriteCommand(0x2BU);
	WriteData(yStart >> 8);
	WriteData(yStart);//Starts at 00h
	WriteData(yEnd >> 8);
	WriteData(yEnd);//Ends at pixel number 320(Width)

	//Start writing the data
	WriteCommand(0x2CU);

	//Set the mode to Data write mode
	HAL_GPIO_WritePin(ILI9341_DC_PORT, ILI9341_DC_PIN, GPIO_PIN_SET);
}

/*******************************************************************************
* @Function Name: ILI9341Pixel()
* @Description: Transmit pixel data
* @input param : uint16_t, uint16_t, color_t
* @return: void
*******************************************************************************/
void ILI9341_write_Pixel(uint16_t x, uint16_t y, colour_t colour)
{
	colour_t beColour = __builtin_bswap16(colour);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&beColour, 2U, 100UL);
}

