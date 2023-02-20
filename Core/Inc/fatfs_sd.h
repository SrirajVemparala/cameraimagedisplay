/******************************************************************************
* @file: fatfs_sd.c
*
* @brief: This file consists of function declarations and macros for SD card operations
*
* @author: Raghu Sai Phani Sriraj Vemparala
* @references: https://github.com/eziya/STM32_SPI_SDCARD/blob/master/STM32F4_HAL_SPI_SDCARD/Src/fatfs_sd.c
*******************************************************************************/
#ifndef INC_FATFS_SD_H_
#define INC_FATFS_SD_H_

#ifndef __FATFS_SD_H
#define __FATFS_SD_H

/*******************************************************************************
 Macros
*******************************************************************************/
/* Definitions for MMC/SDC command */
#define CMD0     (0x40+0)     /* GO_IDLE_STATE */
#define CMD1     (0x40+1)     /* SEND_OP_COND */
#define CMD8     (0x48)     /* SEND_IF_COND */
#define CMD9     (0x40+9)     /* SEND_CSD */
#define CMD10    (0x40+10)    /* SEND_CID */
#define CMD12    (0x40+12)    /* STOP_TRANSMISSION */
#define CMD16    (0x40+16)    /* SET_BLOCKLEN */
#define CMD17    (0x40+17)    /* READ_SINGLE_BLOCK */
#define CMD18    (0x40+18)    /* READ_MULTIPLE_BLOCK */
#define CMD23    (0x40+23)    /* SET_BLOCK_COUNT */
#define CMD24    (0x40+24)    /* WRITE_BLOCK */
#define CMD25    (0x40+25)    /* WRITE_MULTIPLE_BLOCK */
#define CMD41    (0x40+41)    /* SEND_OP_COND (ACMD) */
#define CMD55    (0x40+55)    /* APP_CMD */
#define CMD58    (0x40+58)    /* READ_OCR */
#define SPI_TIMEOUT 1000

/*******************************************************************************
 Function declarations
*******************************************************************************/
DSTATUS SD_disk_initialize (BYTE pdrv);
DSTATUS SD_disk_status (BYTE pdrv);
DRESULT SD_disk_read (BYTE pdrv, BYTE* buff, DWORD sector, UINT count);
DRESULT SD_disk_write (BYTE pdrv, const BYTE* buff, DWORD sector, UINT count);
DRESULT SD_disk_ioctl (BYTE pdrv, BYTE cmd, void* buff);



#endif


#endif /* INC_FATFS_SD_H_ */
