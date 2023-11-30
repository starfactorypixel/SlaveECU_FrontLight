#include "sd.h"
#include <stdio.h>
#include <string.h>

//--------------------------------------------------
// Definitions for MMC/SDC command
#define CMD0	(0x40+0)	// GO_IDLE_STATE
#define CMD1	(0x40+1)	// SEND_OP_COND (MMC)
#define CMD8	(0x40+8)	// SEND_IF_COND
#define CMD9	(0x40+9)	// SEND_CSD
#define CMD12	(0x40+12)	// STOP_TRANSMISSION
#define CMD16	(0x40+16)	// SET_BLOCKLEN
#define CMD17	(0x40+17)	// READ_SINGLE_BLOCK
#define CMD18	(0x40+18)	// READ_MULTIPLE_BLOCK
#define CMD24	(0x40+24)	// WRITE_BLOCK
#define CMD55	(0x40+55)	// APP_CMD
#define CMD58	(0x40+58)	// READ_OCR
#define ACMD41	(0xC0+41)	// SEND_OP_COND (SDC)
//--------------------------------------------------
extern SPI_HandleTypeDef hspi2;
//extern UART_HandleTypeDef huart1;
sd_info_ptr sdinfo;
char str1[60]={0};

SPI_HandleTypeDef *hspi_obj = &hspi2;





void HAL_SPI_ReadFast(uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
{
	SPI_TypeDef *SPIx= hspi_obj->Instance;
	uint32_t tickstart = HAL_GetTick();
	
	__HAL_SPI_ENABLE(hspi_obj);
	while(Size--)
	{
		while( (SPIx->SR & SPI_FLAG_TXE) == 0 )
		{
			if( (HAL_GetTick() - tickstart) >= Timeout ) goto exit;
		}
		*(__IO uint8_t *)&SPIx->DR = 0xFF;
		
		while( (SPIx->SR & SPI_FLAG_RXNE) == 0 )
		{
			if( (HAL_GetTick() - tickstart) >= Timeout ) goto exit;
		}
		*pRxData++ = *(__IO uint8_t *)&SPIx->DR;
	}
	
	exit:
	
	return;
}

void HAL_SPI_WriteFast(uint8_t *pTxData, uint16_t Size, uint32_t Timeout)
{
	SPI_TypeDef *SPIx= hspi_obj->Instance;
	uint32_t tickstart = HAL_GetTick();
	uint8_t RXdummy;
	
	__HAL_SPI_ENABLE(hspi_obj);
	while(Size--)
	{
		while( (SPIx->SR & SPI_FLAG_TXE) == 0 )
		{
			if( (HAL_GetTick() - tickstart) >= Timeout ) goto exit;
		}
		*(__IO uint8_t *)&SPIx->DR = *pTxData++;

		while( (SPIx->SR & SPI_FLAG_RXNE) == 0 )
		{
			if( (HAL_GetTick() - tickstart) >= Timeout ) goto exit;
		}
		RXdummy = *(__IO uint8_t *)&SPIx->DR;
	}
	
	exit:
	
	return;
}

void HAL_SPI_WriteReadFast(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
{
	SPI_TypeDef *SPIx= hspi_obj->Instance;
	uint32_t tickstart = HAL_GetTick();

	__HAL_SPI_ENABLE(hspi_obj);
	while(Size--)
	{
		while( (SPIx->SR & SPI_FLAG_TXE) == 0 )
		{
			if( (HAL_GetTick() - tickstart) >= Timeout ) goto exit;
		}
		*(__IO uint8_t *)&SPIx->DR = *pTxData++;

		while( (SPIx->SR & SPI_FLAG_RXNE) == 0 )
		{
			if( (HAL_GetTick() - tickstart) >= Timeout ) goto exit;
		}
		*pRxData++ = *(__IO uint8_t *)&SPIx->DR;
	}
	
	exit:
	
	return;
}






//-----------------------------------------------
uint8_t SPIx_WriteRead(uint8_t Byte)
{
  uint8_t receivedbyte = 0;
  //if(HAL_SPI_TransmitReceive(hspi_obj,(uint8_t*) &Byte,(uint8_t*) &receivedbyte,1,0x1000)!=HAL_OK)
  //{
  //  //Error();
  //}
  HAL_SPI_WriteReadFast((uint8_t *)&Byte, (uint8_t *)&receivedbyte, 1, 1000);
  
  return receivedbyte;
}

void SPI_SendByte(uint8_t bt)
{
  SPIx_WriteRead(bt);
}

uint8_t SPI_ReceiveByte(void)
{
  uint8_t bt = SPIx_WriteRead(0xFF);
  return bt;
}

inline void SPI_Release(void)
{
  //SPIx_WriteRead(0xFF);
  uint8_t data[1] = {0xFF};
  HAL_SPI_WriteFast(data, 1, 100);
}
//-----------------------------------------------

uint8_t SPI_wait_ready(uint8_t neq, uint8_t *result)
{
	uint16_t cnt = 0;
	
	do {
		HAL_SPI_ReadFast(result, 1, 100);
	} while ( *result != neq && ++cnt < 0xFFFF );
	
	return (cnt == 0xFFFF) ? 0 : 1;
}

uint8_t SD_cmd(uint8_t cmd, uint32_t arg)
{
	uint8_t result;
	uint8_t cnt = 0;
	uint8_t rxtxbuff[6];
	
	// ACMD<n> is the command sequense of CMD55-CMD<n>
	if(cmd & 0x80)
	{
		cmd &= 0x7F;
		result = SD_cmd(CMD55, 0);
		if(result > 1) return result;
	}
	
	// Select the card
	HAL_SPI_ReadFast(rxtxbuff, 2, 100);
	
	cnt = 0x01;							// Dummy CRC + Stop
	if(cmd == CMD0) { cnt = 0x95; }		// Valid CRC for CMD0(0)
	if(cmd == CMD8) { cnt = 0x87; }		// Valid CRC for CMD8(0x1AA)
	
	// Send a command packet
	rxtxbuff[0] = cmd;
	arg = __builtin_bswap32(arg);
	memcpy(rxtxbuff+1, &arg, 4);
	rxtxbuff[5] = cnt;
	HAL_SPI_WriteFast(rxtxbuff, 6, 1000);
	
	// Ждём ответ R1, в котором старший бит всегда 0. На шине до этого 0xFF
	cnt = 10;
	do {
		HAL_SPI_ReadFast(rxtxbuff, 1, 100);
	} while ( (rxtxbuff[0] & 0x80) && --cnt);
	
	return rxtxbuff[0];
}

//-----------------------------------------------
void SD_PowerOn(void)
{
 // Timer1 = 0;
 // while(Timer1<2) //��� 20 ����������, ��� ����, ����� ���������� �����������������
 // ;
}
//-----------------------------------------------


// http://chlazza.nfshost.com/sdcardinfo.html
// https://www.st.com/resource/en/application_note/an5595-spc58xexspc58xgx-multimedia-card-via-spi-interface-stmicroelectronics.pdf
// http://www.edproject.co.uk/18Series14.html

uint8_t SD_Read_Block(uint8_t *buff, uint32_t lba)
{
	static const uint16_t block_size = 512U;
	uint8_t result;
	uint8_t rxtxbuff[2];
	
	// Отправляем команду CMD17 (READ_SINGLE_BLOCK)
	result = SD_cmd(CMD17, lba);
	if(result != 0x00) return 5;
	
	// Непонимаю зачем это тут, но это уменьшает время ожидания данных на ~1мс.
	HAL_SPI_ReadFast(rxtxbuff, 1, 100);
	
	// Ждём токен начала данных
	if( SPI_wait_ready(0xFE, &result) == 0 ) return 5;

	// Читаем 512 байт
	HAL_SPI_ReadFast(buff, block_size, 1000);

	// Читаем 2 байта CRC
	HAL_SPI_ReadFast(rxtxbuff, 2, 100);

	// Если используем CMD18 (READ_MULTIPLE_BLOCK), то останавливаем передачу.
	//SD_cmd(CMD12, 0x00000000);

	return 0;
}

uint8_t SD_Write_Block (uint8_t *buff, uint32_t lba)
{
  uint8_t result;
  uint16_t cnt;
  result=SD_cmd(CMD24,lba); //CMD24 ������� ��� 51 � 97-98
  if (result!=0x00) return 6; //�����, ���� ��������� �� 0x00
  SPI_Release();
  SPI_SendByte (0xFE); //������ ������
  for (cnt=0;cnt<512;cnt++) SPI_SendByte(buff[cnt]); //������
  SPI_Release(); //��������� ���������� �����
  SPI_Release();
  result=SPI_ReceiveByte();
  if ((result&0x05)!=0x05) return 6; //�����, ���� ��������� �� 0x05 (������� ��� 111)
  cnt=0;
  do { //���� ��������� ��������� BUSY
    result=SPI_ReceiveByte();
    cnt++;
  } while ( (result!=0xFF)&&(cnt<0xFFFF) );
  if (cnt>=0xFFFF) return 6;
  return 0;
}

uint8_t sd_ini(void)
{
	uint8_t i, cmd;
  int16_t tmr;
  uint32_t temp;

  sdinfo.type = 0;
	uint8_t ocr[4];
	temp = hspi_obj->Init.BaudRatePrescaler;
	hspi_obj->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; //156.25 kbbs
	HAL_SPI_Init(hspi_obj);
	SS_SD_DESELECT();
	for(i=0;i<10;i++) //80 ��������� (�� ����� 74) ������� ��� 91
		SPI_Release();
	hspi_obj->Init.BaudRatePrescaler = temp;
	HAL_SPI_Init(hspi_obj);
	SS_SD_SELECT();
  if (SD_cmd(CMD0, 0) == 1) // Enter Idle state
  {
		SPI_Release();
		if (SD_cmd(CMD8, 0x1AA) == 1) // SDv2
		{
			for (i = 0; i < 4; i++) ocr[i] = SPI_ReceiveByte();
			//sprintf(str1,"OCR: 0x%02X 0x%02X 0x%02X 0x%02X\r\n",ocr[0],ocr[1],ocr[2],ocr[3]);
			//HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
			// Get trailing return value of R7 resp
			if (ocr[2] == 0x01 && ocr[3] == 0xAA) // The card can work at vdd range of 2.7-3.6V
			{
				  for (tmr = 12000; tmr && SD_cmd(ACMD41, 1UL << 30); tmr--)
						; // Wait for leaving idle state (ACMD41 with HCS bit)
					if (tmr && SD_cmd(CMD58, 0) == 0) { // Check CCS bit in the OCR
					for (i = 0; i < 4; i++) ocr[i] = SPI_ReceiveByte();
					//sprintf(str1,"OCR: 0x%02X 0x%02X 0x%02X 0x%02X\r\n",ocr[0],ocr[1],ocr[2],ocr[3]);
					//HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
					sdinfo.type = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2; // SDv2 (HC or SC)
				}
			}
		}
		else //SDv1 or MMCv3
		{
			if (SD_cmd(ACMD41, 0) <= 1)
			{
				sdinfo.type = CT_SD1; cmd = ACMD41; // SDv1
			}
			else
			{
				sdinfo.type = CT_MMC; cmd = CMD1; // MMCv3
			}
			for (tmr = 25000; tmr && SD_cmd(cmd, 0); tmr--) ; // Wait for leaving idle state
			if (!tmr || SD_cmd(CMD16, 512) != 0) // Set R/W block length to 512
			sdinfo.type = 0;
		}
  }
  else
  {
    return 1;
  }
  //sprintf(str1,"Type SD: 0x%02X\r\n",sdinfo.type);
  //HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);	
  return 0;
}
//-----------------------------------------------
