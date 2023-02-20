/******************************************************************************
* @file: main.c
*
* @brief: This file consists of start of the code
*
* @author: Raghu Sai Phani Sriraj Vemparala
* @references: http://www.ece.ualberta.ca/~elliott/ee552/studentAppNotes/2003_w/misc/bmp_file_format/bmp_file_format.htm
*******************************************************************************/
/*******************************************************************************
Header files
*******************************************************************************/
#include "main.h"
#include "fatfs.h"
#include "ili9341.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
//#include "touch.h"
#include "fatfs_sd.h"
#include "string.h"

/*******************************************************************************
 Macros
*******************************************************************************/
#define LCD_WIDTH 240
#define LCD_HEIGHT 320

/*******************************************************************************
 Global variables
*******************************************************************************/
/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_tx;
char buff[40];

/*******************************************************************************
 Function declarations
*******************************************************************************/
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/*******************************************************************************
 Global variables
*******************************************************************************/
FATFS fs;  // file system
FIL file; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

/*******************************************************************************
* @Function Name: Image_display()
* @Description: Extract the image and transfer it to LCD
* @input param : void
* @return: int
*******************************************************************************/
int Image_display(const char* fname)
{
    FIL file;
    unsigned int data_read;
    uint8_t header[34];
    //Open the file
    FRESULT res = f_open(&file, fname, FA_READ);
    if(res != FR_OK)
    {
    	printf("Open operation failed");
        return 0;
    }
    //Read 34 bytes of header from the file
    res = f_read(&file, header, sizeof(header), &data_read);
    if(res != FR_OK)
    {
    	printf("read failed");
        f_close(&file);
        return 0;
    }
    printf("Header value is%d",header[0]);
    //Check if the file format is BMP
    if((header[0] != 0x42) || (header[1] != 0x4D))
    {
    	printf("Not a BITMAP file");
        f_close(&file);
        return 0;
    }
    //Obtain important information from the header
    uint32_t imageOffset = header[10] | (header[11] << 8) | (header[12] << 16) | (header[13] << 24);
    uint32_t imageWidth = header[18] | (header[19] << 8) | (header[20] << 16) | (header[21] << 24);
    uint32_t imageHeight = header[22] | (header[23] << 8) | (header[24] << 16) | (header[25] << 24);
    uint16_t imagePlanes = header[26] | (header[27] << 8);
    uint16_t imageBitsPerPixel = header[28] | (header[29] << 8);
    //Check if the width and height of the image fits the LCD
    if((imageWidth != ILI9341_LCD_WIDTH) || (imageHeight != ILI9341_LCD_HEIGHT))
    {
        f_close(&file);
        return 0;
    }
    //Check if image planes is  1 and bits per pixel is  24
    if((imagePlanes != 1) || (imageBitsPerPixel != 24))
    {
        f_close(&file);
        return 0;
    }
    //Go to array place where image data is stored
    res = f_lseek(&file, imageOffset);
    if(res != FR_OK)
    {

        f_close(&file);
        return 0;
    }

    // row size is aligned to 4 bytes
    uint8_t imageRow[(ILI9341_LCD_WIDTH * 3)];
    for(uint32_t y = 0; y < imageHeight; y++)
    {
        uint32_t rowIdx = 0;
        res = f_read(&file, imageRow, sizeof(imageRow), &data_read);
        if(res != FR_OK)
        {
            f_close(&file);
            return 0;
        }

        for(uint32_t x = 0; x < imageWidth; x++)
        {
            uint8_t blue = imageRow[rowIdx++];
            uint8_t green = imageRow[rowIdx++];
            uint8_t red = imageRow[rowIdx++];
            //Converting to 565 format
            uint16_t color565 = (((red & 0xF8) << 8) | ((green & 0xFC) << 3) | ((blue & 0xF8) >> 3));
            ILI9341_write_Pixel(x, imageHeight - y - 1, color565);
        }
    }

    res = f_close(&file);
    if(res != FR_OK) {
        return 0;
    }

    return 0;
}
/* USER CODE END 0 */

/*******************************************************************************
* @Function Name: main()
* @Description: starting of the execution
* @input param : void
* @return: int
*******************************************************************************/

int main(void)
{

	int i = 0;
    int flag=0;//used to go to the next picture
	int flag1=0;//used to revert back to the starting picture
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  //Start the ILI9341 initialization
  ili9341_start();
  //Set the address position from where the data is to be written on LCD
  setaddress(0,0,LCD_WIDTH-1,LCD_HEIGHT-1);
     FRESULT res = f_mount(&fs, "", 0);//mount SD card
     if(res != FR_OK)
     {
    	 printf("Mounting_Successful");
         return 0;
     }
     while (1)
     {
    	 //Check if PinB1 is set
    		 if( HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1 )== GPIO_PIN_SET)
    		 {
    			 if(flag==0)
    			 {
    				 	 i++;
    				 	 flag=1;
    				 	 flag1=0;
    	  	     }
    		 }
    		 //check if PINE7 is set
    		 if( HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_7 )== GPIO_PIN_SET)
    		  {
    			 if(flag==0)
    			 {
    		 			 i--;
    		 			 flag=1;
    		 			 flag1=0;
    		     }
    		  }
    		 if(i == 1)//Based on the push button press the data is changed
    		 {
                if(flag1==0)
                {
                	Image_display("design.bmp");
    		        HAL_Delay(1000);
    		        flag=0;
    		        flag1=1;
    		    }
    		 }
    		 if(i == 2)
    		 {
    			 if(flag1==0)
    			 {
    				 Image_display("cartoon.bmp");
    				 HAL_Delay(1000);
    				 flag=0;
    				 flag1=1;
    			 }
    		 }
    		 if(i == 3)
    		 {
    			 if(flag1==0)
    			 {
    				 Image_display("purpleflowers.bmp");
    				 HAL_Delay(1000);
    				 flag=0;
    				 flag1=1;
    			 }
    		 }
    		 if(i == 4)
    		 {
    			 if(flag1==0)
    			 {
    				 Image_display("rajesh.bmp");
    		  HAL_Delay(1000);
    		  flag=0;
    		  flag1=1;
    			 }
    		 }
    		 if(i == 5)
    		 {
    			 if(flag1==0)
    			 {
    				 Image_display("sriraj.bmp");
    		  HAL_Delay(1000);
    		  flag=0;
    		  flag1=1;
    			 }
    		 }
    		 if(i == 6)
    		 {
    			 if(flag1==0)
    			{
    				 Image_display("gokucorrect.bmp");
    		  HAL_Delay(1000);
    		  flag=0;
    		  flag1=1;
    		  i=0;
    		 }
    		 }
    		 if(i<=0)
    		 {
    			 i=0;
    			 flag=0;

    		 }
    		 if(i>=6)
    		 {
    			 i=0;
    			 flag=0;
    		 }
  /* USER CODE END 3 */
     }
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SD_CS_Pin|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : SD_CS_Pin PC4 PC5 */
  GPIO_InitStruct.Pin = SD_CS_Pin|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
