/******************************************************************************
* @file: fatfs_sd.c
*
* @brief: This file consists of functions for SD card operations
*
* @author: Raghu Sai Phani Sriraj Vemparala
* @references: https://github.com/eziya/STM32_SPI_SDCARD/blob/master/STM32F4_HAL_SPI_SDCARD/Src/fatfs_sd.c
*******************************************************************************/

/*******************************************************************************
Header files
*******************************************************************************/
#include "stm32f4xx_hal.h"

#include "diskio.h"
#include "fatfs_sd.h"

/*******************************************************************************
 Global variables
*******************************************************************************/
extern SPI_HandleTypeDef hspi2;
extern volatile uint8_t Timer1, Timer2;

static volatile DSTATUS Stat = STA_NOINIT;
static uint8_t CardType;
static uint8_t PowerFlag = 0;

/*******************************************************************************
 Macros
*******************************************************************************/
#define SD_CS_GPIO_Port GPIOB
#define SD_CS_Pin GPIO_PIN_12

#define TRUE  1
#define FALSE 0
#define bool BYTE

/*******************************************************************************
* @Function Name: SELECT()
* @Description: chip select pin set to 0
* @input param : void
* @return: void
*******************************************************************************/
static void SELECT(void)
{
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
}

/*******************************************************************************
* @Function Name: DESELECT()
* @Description: chip select pin set to 1
* @input param : void
* @return: void
*******************************************************************************/
/* SPI Chip Deselect */
static void DESELECT(void)
{
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
}

/*******************************************************************************
* @Function Name: SPI_TxByte()
* @Description: Transmit SPI data
* @input param :unsigned char
* @return: static void
*******************************************************************************/
static void SPI_TxByte(BYTE data)
{
  while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
  HAL_SPI_Transmit(&hspi2, &data, 1, SPI_TIMEOUT);
}

/*******************************************************************************
* @Function Name: SPI_RxByte()
* @Description: Receive SPI data
* @input param : void
* @return: static uint8_t
*******************************************************************************/
static uint8_t SPI_RxByte(void)
{
  uint8_t dummy, data;
  dummy = 0xFF;
  data = 0;

  while ((HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY));
  HAL_SPI_TransmitReceive(&hspi2, &dummy, &data, 1, SPI_TIMEOUT);

  return data;
}

/*******************************************************************************
* @Function Name: SPI_RxBytePtr()
* @Description: Receive SPI data and store the address
* @input param : uint8_t*
* @return: static void
*******************************************************************************/
static void SPI_RxBytePtr(uint8_t *buff)
{
  *buff = SPI_RxByte();
}

/*******************************************************************************
* @Function Name: SD_ReadyWait()
* @Description: Wait till the data is received
* @input param : uint8_t
* @return: static void
*******************************************************************************/
static uint8_t SD_ReadyWait(void)
{
  uint8_t res;

  /* 500ms timer */
  Timer2 = 50;
  SPI_RxByte();

  do
  {
    /* 0xFF  SPI Rx*/
    res = SPI_RxByte();
  } while ((res != 0xFF) && Timer2);

  return res;
}
/*******************************************************************************
* @Function Name: SD_PowerOn()
* @Description: Enable SD card
* @input param : void
* @return: static void
*******************************************************************************/
static void SD_PowerOn(void)
{
  uint8_t cmd_arg[6];

  /* Deselect SPI communication */
  DESELECT();
  //send the cmd 6 to 7 times to make sd card reset
  for(int i = 0; i < 10; i++)
  {
    SPI_TxByte(0xFF);
  }

  /* SPI Chips Select */
  SELECT();

  /*Go_to_IDLE_STATE  */
  cmd_arg[0] = (CMD0 | 0x40);
  cmd_arg[1] = 0;
  cmd_arg[2] = 0;
  cmd_arg[3] = 0;
  cmd_arg[4] = 0;
  cmd_arg[5] = 0x95;//CRC
//Transmit the data
  for (int i = 0; i < 6; i++)
  {
    SPI_TxByte(cmd_arg[i]);//Transmit all the data
  }

  while ((SPI_RxByte() != 0x01))//Verify if the acknowledge is correct
  {

  }

  DESELECT();


  PowerFlag = 1;
}
/*******************************************************************************
* @Function Name: SD_PowerOff()
* @Description: Reset the SD card
* @input param : void
* @return: static void
*******************************************************************************/
static void SD_PowerOff(void)
{
  PowerFlag = 0;
}

/*******************************************************************************
* @Function Name: SD_CheckPower()
* @Description: Get the power status on SD card
* @input param : void
* @return: static void
*******************************************************************************/
static uint8_t SD_CheckPower(void)
{
  /*  0=off, 1=on */
  return PowerFlag;
}

/*******************************************************************************
* @Function Name: SD_RxDataBlock()
* @Description: Get complete block information
* @input param : unsigned char *, unsigned int
* @return: static bool
*******************************************************************************/
static bool SD_RxDataBlock(BYTE *buff, UINT btr)
{
  uint8_t token;

  /* 100ms*/
  Timer1 = 10;

  do
  {
    token = SPI_RxByte();
  } while((token == 0xFF) && Timer1);

  /* 0xFE  Token*/
  if(token != 0xFE)
    return FALSE;


  do
  {
    SPI_RxBytePtr(buff++);
    SPI_RxBytePtr(buff++);
  } while(btr -= 2);

  SPI_RxByte(); /* CRC */
  SPI_RxByte();

  return TRUE;
}

/*******************************************************************************
* @Function Name: SD_TxDataBlock()
* @Description: Get complete block information
* @input param : unsigned char *, unsigned int
* @return: static bool
*******************************************************************************/
#if _READONLY == 0
static bool SD_TxDataBlock(const BYTE *buff, BYTE token)
{
  uint8_t resp, wc;//response, wait
  uint8_t i = 0;

  /* SD*/
  if (SD_ReadyWait() != 0xFF)
    return FALSE;

  SPI_TxByte(token);

  if (token != 0xFD)
  {
    wc = 0;

    do
    {
      SPI_TxByte(*buff++);
      SPI_TxByte(*buff++);
    } while (--wc);

    SPI_RxByte();
    SPI_RxByte();

    while (i <= 64)
    {
      resp = SPI_RxByte();

      if ((resp & 0x1F) == 0x05)
        break;

      i++;
    }

    /* SPI Clear */
    while (SPI_RxByte() == 0);
  }

  if ((resp & 0x1F) == 0x05)
    return TRUE;
  else
    return FALSE;
}
#endif /* _READONLY */


/*******************************************************************************
* @Function Name: SD_SendCmd()
* @Description:  Transmit command
* @input param : unsigned char , unsigned long
* @return: static unsigned char
*******************************************************************************/
static BYTE SD_SendCmd(BYTE cmd, DWORD arg)
{
  uint8_t crc, res;

  /* Wait and receive the data*/
  if (SD_ReadyWait() != 0xFF)
    return 0xFF;

//Send data and the argument received
  SPI_TxByte(cmd); 			/* Command */
  SPI_TxByte((BYTE) (arg >> 24)); 	/* Argument[31..24] */
  SPI_TxByte((BYTE) (arg >> 16)); 	/* Argument[23..16] */
  SPI_TxByte((BYTE) (arg >> 8)); 	/* Argument[15..8] */
  SPI_TxByte((BYTE) arg); 		/* Argument[7..0] */

  crc = 0;
  if (cmd == CMD0)
    crc = 0x95; /* CRC for CMD0(0) */

  if (cmd == CMD8)
    crc = 0x87; /* CRC for CMD8(0x1AA) */

  /* CRC */
  SPI_TxByte(crc);

  /* CMD12 Stop Reading */
  if (cmd == CMD12)
    SPI_RxByte();

  uint8_t n = 10;
  do
  {
    res = SPI_RxByte();
  } while ((res & 0x80) && --n);

  return res;
}


/*******************************************************************************
* @Function Name: SD_disk_initialize()
* @Description: Initialize the SD card
* @input param : unsigned char
* @return: unsigned char
*******************************************************************************/
DSTATUS SD_disk_initialize(BYTE drv)
{
  uint8_t n, type, ocr[4];


  if(drv)
    return STA_NOINIT;

  /* SD*/
  if(Stat & STA_NODISK)
    return Stat;

  /* SD Power On */
  SD_PowerOn();

  /* SPI Chip Select */
  SELECT();

  /* SD */
  type = 0;

  /* Idle */
  if (SD_SendCmd(CMD0, 0) == 1)
  {
    /* Timer 1*/
    Timer1 = 100;

    /* SD */
    if (SD_SendCmd(CMD8, 0x1AA) == 1)
    {
      for (n = 0; n < 4; n++)
      {
        ocr[n] = SPI_RxByte();
      }
      /*check if echo is received properly*/
      if (ocr[2] == 0x01 && ocr[3] == 0xAA)
      {
        /* Check the voltage level and also check if the card is HC */
        do {
          if ((SD_SendCmd(CMD55, 0) <= 1) &&( SD_SendCmd(CMD41, 1 << 30)) == 0)
            break; /* ACMD41 with HCS bit */
        } while (Timer1);

        if (Timer1 && SD_SendCmd(CMD58, 0) == 0)
        {
          /* Check CCS bit */
          for (n = 0; n < 1; n++)
          {
            ocr[n] = SPI_RxByte();
          }
          //Check if Card is HSC and CCS is set to 1
          type = (ocr[0] & 0x40) ? 6 : 2;
        }
      }
    }
    else
    {

    }
}
  CardType = type;

  DESELECT();

  SPI_RxByte(); /* Idle (Release DO) */

  if (type)
  {
    /* Clear STA_NOINIT */
    Stat &= ~STA_NOINIT;
  }
  else
  {
    /* Initialization failed */
    SD_PowerOff();
  }

  return Stat;
}

/*******************************************************************************
* @Function Name: SD_disk_status()
* @Description: get disk status
* @input param : unsigned char
* @return: unsigned char
*******************************************************************************/
DSTATUS SD_disk_status(BYTE drv)
{
  if (drv)
    return STA_NOINIT;

  return Stat;
}

/*******************************************************************************
* @Function Name: SD_disk_read()
* @Description: read data from disk
* @input param : unsigned char, unsigned char*, unsigned long, unsigned int
* @return: enum
*******************************************************************************/
DRESULT SD_disk_read(BYTE pdrv, BYTE* buff, DWORD sector, UINT count)
{
  if (pdrv || !count)
    return RES_PARERR;

  if (Stat & STA_NOINIT)
    return RES_NOTRDY;

  if (!(CardType & 4))
    sector *= 512;      /*sector Byte addressing */

  SELECT();

  if (count == 1)
  {
    /*Tranmit command 17*/
    if ((SD_SendCmd(CMD17, sector) == 0) && SD_RxDataBlock(buff, 512))
      count = 0;
  }
  else
  {
    /*Tranmit command 18 */
    if (SD_SendCmd(CMD18, sector) == 0)
    {
      do {
        if (!SD_RxDataBlock(buff, 512))//Read block of 512 bytes
          break;

        buff += 512;
      } while (--count);

      /* STOP_TRANSMISSION*/
      SD_SendCmd(CMD12, 0);
    }
  }

  DESELECT();
  SPI_RxByte(); /* Idle (Release DO) */

  return count ? RES_ERROR : RES_OK;
}

/*******************************************************************************
* @Function Name: SD_disk_write()
* @Description: write data to disk
* @input param : unsigned char, unsigned char*, unsigned long, unsigned int
* @return: enum
*******************************************************************************/
#if _READONLY == 0
DRESULT SD_disk_write(BYTE pdrv, const BYTE* buff, DWORD sector, UINT count)
{
  if (pdrv || !count)
    return RES_PARERR;

  if (Stat & STA_NOINIT)
    return RES_NOTRDY;

  if (Stat & STA_PROTECT)
    return RES_WRPRT;

  if (!(CardType & 4))
    sector *= 512; /*sector Byte addressing */

  SELECT();

  if (count == 1)
  {
    if ((SD_SendCmd(CMD24, sector) == 0) && SD_TxDataBlock(buff, 0xFE))
      count = 0;
  }
  else
  {

    if (CardType & 2)
    {
      SD_SendCmd(CMD55, 0);
      SD_SendCmd(CMD23, count); /* ACMD23 */
    }

    if (SD_SendCmd(CMD25, sector) == 0)
    {
      do {
        if(!SD_TxDataBlock(buff, 0xFC))
          break;

        buff += 512;
      } while (--count);

      if(!SD_TxDataBlock(0, 0xFD))
      {
        count = 1;
      }
    }
  }

  DESELECT();
  SPI_RxByte();

  return count ? RES_ERROR : RES_OK;
}
#endif /* _READONLY */

/*******************************************************************************
* @Function Name: SD_disk_ioctl()
* @Description: Used to send SD card commands
* @input param : unsigned char, unsigned char, void
* @return: enum
*******************************************************************************/
DRESULT SD_disk_ioctl(BYTE drv, BYTE ctrl, void *buff)
{
  DRESULT res;
  BYTE n, csd[16], *ptr = buff;
  WORD csize;

  if (drv)
    return RES_PARERR;

  res = RES_ERROR;

  if (ctrl == CTRL_POWER)
  {
    switch (*ptr)
    {
    case 0:
      if (SD_CheckPower())
        SD_PowerOff();          /* Power Off */
      res = RES_OK;
      break;
    case 1:
      SD_PowerOn();             /* Power On */
      res = RES_OK;
      break;
    case 2:
      *(ptr + 1) = (BYTE) SD_CheckPower();
      res = RES_OK;             /* Power Check */
      break;
    default:
      res = RES_PARERR;
    }
  }
  else
  {
    if (Stat & STA_NOINIT)
      return RES_NOTRDY;

    SELECT();

    switch (ctrl)
    {
    case GET_SECTOR_COUNT:
      /* SD Sector (DWORD) */
      if ((SD_SendCmd(CMD9, 0) == 0) && SD_RxDataBlock(csd, 16))
      {
        if ((csd[0] >> 6) == 1)
        {
          /* SDC ver 2.00 */
          csize = csd[9] + ((WORD) csd[8] << 8) + 1;
          *(DWORD*) buff = (DWORD) csize << 10;
        }
        else
        {
          /* MMC or SDC ver 1.XX */
          n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
          csize = (csd[8] >> 6) + ((WORD) csd[7] << 2) + ((WORD) (csd[6] & 3) << 10) + 1;
          *(DWORD*) buff = (DWORD) csize << (n - 9);
        }

        res = RES_OK;
      }
      break;

    case GET_SECTOR_SIZE:
      /* (WORD) */
      *(WORD*) buff = 512;
      res = RES_OK;
      break;

    case CTRL_SYNC:
      /* */
      if (SD_ReadyWait() == 0xFF)
        res = RES_OK;
      break;

    case MMC_GET_CSD:
      /* CSD (16 bytes) */
      if (SD_SendCmd(CMD9, 0) == 0 && SD_RxDataBlock(ptr, 16))
        res = RES_OK;
      break;

    case MMC_GET_CID:
      /* CID (16 bytes) */
      if (SD_SendCmd(CMD10, 0) == 0 && SD_RxDataBlock(ptr, 16))
        res = RES_OK;
      break;

    case MMC_GET_OCR:
      /* OCR (4 bytes) */
      if (SD_SendCmd(CMD58, 0) == 0)
      {
        for (n = 0; n < 4; n++)
        {
          *ptr++ = SPI_RxByte();
        }

        res = RES_OK;
      }

    default:
      res = RES_PARERR;
    }

    DESELECT();
    SPI_RxByte();
  }

  return res;
}
