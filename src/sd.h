#ifndef SD_H_
#define SD_H_
//--------------------------------------------------
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#warning It doesn't work if '-O2'. Fast fix..
#pragma GCC optimize("-Ofast")

//--------------------------------------------------
#define CS_SD_GPIO_PORT GPIOA
#define CS_SD_PIN GPIO_PIN_8
#define SS_SD_SELECT() HAL_GPIO_WritePin(CS_SD_GPIO_PORT, CS_SD_PIN, GPIO_PIN_RESET)
#define SS_SD_DESELECT() HAL_GPIO_WritePin(CS_SD_GPIO_PORT, CS_SD_PIN, GPIO_PIN_SET)
//--------------------------------------------------
/* Card type flags (CardType) */
#define CT_MMC 0x01 /* MMC ver 3 */
#define CT_SD1 0x02 /* SD ver 1 */
#define CT_SD2 0x04 /* SD ver 2 */
#define CT_SDC (CT_SD1|CT_SD2) /* SD */
#define CT_BLOCK 0x08 /* Block addressing */
//--------------------------------------------------
typedef struct sd_info {
  volatile uint8_t type;//��� �����
} sd_info_ptr;
//--------------------------------------------------
void SD_PowerOn(void);
uint8_t sd_ini(void);
void SPI_Release(void);
uint8_t SD_Read_Block (uint8_t *buff, uint32_t lba);
uint8_t SD_Write_Block (uint8_t *buff, uint32_t lba);
uint8_t SPI_wait_ready(uint8_t neq, uint8_t *result);
//--------------------------------------------------
#endif /* SD_H_ */
