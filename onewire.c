/**
 * @file	onewire.c
 * @brief	������� ���������� 1-wire
 */

#include "onewire.h"

#include "stm32f10x_rcc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"

#ifdef OW_USART1

#undef OW_USART2
#undef OW_USART3
#undef OW_USART4

#define OW_USART 		USART1
#define OW_DMA_CH_RX 	DMA1_Channel5
#define OW_DMA_CH_TX 	DMA1_Channel4
#define OW_DMA_FLAG		DMA1_FLAG_TC5

#endif


#ifdef OW_USART2

#undef OW_USART1
#undef OW_USART3
#undef OW_USART4

#define OW_USART 		USART2
#define OW_DMA_CH_RX 	DMA1_Channel6
#define OW_DMA_CH_TX 	DMA1_Channel7
#define OW_DMA_FLAG		DMA1_FLAG_TC6

#endif


// ����� ��� ������/�������� �� 1-wire
unsigned char ow_buf[8];

#define OW_0	0x00
#define OW_1	0xff
#define OW_R_1	0xff

/**
 * @brief	����������� ���� ���� � ������, ��� �������� ����� USART
 * @param	ow_byte - ����, ������� ���� �������������
 * @param	ow_bits - ������ �� �����, �������� �� ����� 8 ����
 * @retval	���
 */
void ow_tobits(unsigned char ow_byte, unsigned char *ow_bits) {
	for (unsigned char i=0; i<8; i++) {
		if (ow_byte & 0x01) {
			*ow_bits = OW_1;
		} else {
			*ow_bits = OW_0;
		}
		ow_bits++;
		ow_byte = ow_byte >> 1;
	}
}

/**
 * @brief	�������� �������������� - �� ����, ��� �������� ����� USART ����� ���������� ����
 * @param	ow_bits - ������ �� �����, �������� �� ����� 8 ����
 * @retval	����, ��������� �� �����
 */
unsigned char ow_tobyte(unsigned char *ow_bits) {
	unsigned char ow_byte;
	ow_byte = 0;
	for (unsigned char i=0; i<8; i++) {
		ow_byte = ow_byte >> 1;
		if (*ow_bits == OW_R_1) {
			ow_byte |= 0x80;
		}
		ow_bits++;
	}
	return ow_byte;
}

/**
 * @brief	�������������� USART � DMA
 * @param	���
 * @retval	���
 */
void ow_init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStructure;

	if (OW_USART == USART1) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

		// USART TX
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
		/*
		 * ����� ������� �� ����� ����������� ��������.
		 * � ������ ������ ������� ����� ���������� �������, ������/������ �� ����� Tx, �������������� �����,
		 *  ������� ������ GPIO_Mode_AF_OD.
		 * (����� GPIO_Mode_AF_PP �������� ��������� ������ ��� ������ �������� � ���� ����� Tx � Rx ����������� ����� ����)
		 */
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStruct);

		/*
		 * � ������ ������� ���������� ����� Rx �� ����� (������/������ �� ����� Tx, �������������� �����)
		 */
		// USART RX
		//GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;//����� RX
		//GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		//GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		//GPIO_Init(GPIOA, &GPIO_InitStruct);

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	}

	if (OW_USART == USART2) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,
				ENABLE);

		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStruct);

		//GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;//����� RX
		//GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		//GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		//GPIO_Init(GPIOA, &GPIO_InitStruct);

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	}

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

	USART_Init(OW_USART, &USART_InitStructure);
	USART_Cmd(OW_USART, ENABLE);

    // ����� ������� ���������� ������ USART � �������������� ������
    USART_HalfDuplexCmd(OW_USART, ENABLE);
}

/**
 * @brief	������������ ����� � �������� �� ������� ��������� �� ����
 * @param	���
 * @retval	������ ����������
 */
unsigned char ow_reset(void) {
	unsigned char ow_presence;
	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(OW_USART, &USART_InitStructure);

	// ���������� 0xf0 �� �������� 9600
	USART_ClearFlag(OW_USART, USART_FLAG_TC);
	USART_SendData(OW_USART, 0xf0);
	while (USART_GetFlagStatus(OW_USART, USART_FLAG_TC) == RESET) {
#ifdef OW_GIVE_TICK_RTOS
		taskYIELD();
#endif
	}

	ow_presence = USART_ReceiveData(OW_USART);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(OW_USART, &USART_InitStructure);

	if (ow_presence != 0xf0) {
		return OW_OK;
	}

	return OW_NO_DEVICE;
}

/**
 * @brief	��������� ������� � ����� 1-wire
 * @param	sendReset - �������� RESET � ������ �������
 * 			(OW_SEND_RESET ��� OW_NO_RESET)
 * @param	command - ������ ����, ���������� � ����.
 * 			���� ����� ������ - ���������� OW_READ_SLOTH
 * @param	cLen - ����� ������ ������, ������� ���� ��������� � ����
 * @param	data - ���� ��������� ������, �� ������ �� ����� ��� ������
 * @param	dLen - ����� ������ ��� ������. ����������� �� ����� ���� �����
 * @param	readStart - � ������ ������� �������� �������� ������ (���������� � 0),
 * 			����� ������� OW_NO_READ, ����� ����� �� �������� data � dLen
 * @retval	������ ����������
 */
unsigned char ow_send(unsigned char sendReset, char *command, unsigned char cLen,
		unsigned char *data, unsigned char dLen, unsigned char readStart) {

	// ���� ��������� ����� - ���������� � ��������� �� ������� ���������
	if (sendReset == OW_SEND_RESET) {
		if (ow_reset() == OW_NO_DEVICE) {
			return OW_NO_DEVICE;
		}
	}

	while (cLen > 0) {

		ow_tobits(*command, ow_buf);
		command++;
		cLen--;

		DMA_InitTypeDef DMA_InitStructure;

		// DMA �� ������
		DMA_DeInit(OW_DMA_CH_RX);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(OW_USART->DR);
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ow_buf;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
		DMA_InitStructure.DMA_BufferSize = 8;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
		DMA_Init(OW_DMA_CH_RX, &DMA_InitStructure);

		// DMA �� ������
		DMA_DeInit(OW_DMA_CH_TX);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(OW_USART->DR);
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ow_buf;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
		DMA_InitStructure.DMA_BufferSize = 8;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
		DMA_Init(OW_DMA_CH_TX, &DMA_InitStructure);

		// ����� ����� ��������
		USART_ClearFlag(OW_USART, USART_FLAG_RXNE | USART_FLAG_TC | USART_FLAG_TXE);
		USART_DMACmd(OW_USART, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
		DMA_Cmd(OW_DMA_CH_RX, ENABLE);
		DMA_Cmd(OW_DMA_CH_TX, ENABLE);

		// ����, ���� �� ������ 8 ����
		while (DMA_GetFlagStatus(OW_DMA_FLAG) == RESET){
#ifdef OW_GIVE_TICK_RTOS
			taskYIELD();
#endif
		}

		// ��������� DMA
		DMA_Cmd(OW_DMA_CH_TX, DISABLE);
		DMA_Cmd(OW_DMA_CH_RX, DISABLE);
		USART_DMACmd(OW_USART, USART_DMAReq_Tx | USART_DMAReq_Rx, DISABLE);

		// ���� ����������� ������ ����-�� ����� - ������� �� � �����
		if (readStart == 0 && dLen > 0) {
			*data = ow_tobyte(ow_buf);
			data++;
			dLen--;
		} else {
			if (readStart != OW_NO_READ) {
				readStart--;
			}
		}
	}

	return OW_OK;
}

/**
 * @brief	������������ ����� � ����� Tx
 * @param	���
 * @retval	���
 */
void ow_out_set_as_tx_pin(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	if (OW_USART == USART1) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

		// USART TX
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    if (OW_USART == USART2) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

        GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    if (OW_USART == USART3) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

/**
 * @brief	������������ ����� � ����� Power
 * @param	���
 * @retval	���
 */
void ow_out_set_as_power_pin(void){
    GPIO_InitTypeDef GPIO_InitStruct;
    if (OW_USART == USART1) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
        // GPIO
        //GPIO_SetBits(GPIOA, GPIO_Pin_9);
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIOA->ODR|=GPIO_Pin_9;
        GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    if (OW_USART == USART2) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
        GPIO_SetBits(GPIOA , GPIO_Pin_2);
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    if (OW_USART == USART3) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
        GPIO_SetBits(GPIOB , GPIO_Pin_10);
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

/**
 * @brief	���������� ��������� ����� ���
 * @param	num_bits - ����� ��� ��� ������
 * @retval	���
 */
void ow_send_bits(unsigned char num_bits) {
	DMA_InitTypeDef DMA_InitStructure;

	// DMA �� ������
	DMA_DeInit(OW_DMA_CH_RX);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(OW_USART->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ow_buf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = num_bits;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(OW_DMA_CH_RX, &DMA_InitStructure);

	// DMA �� ������
	DMA_DeInit(OW_DMA_CH_TX);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(OW_USART->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ow_buf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = num_bits;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(OW_DMA_CH_TX, &DMA_InitStructure);

	// ����� ����� ��������
	USART_ClearFlag(OW_USART, USART_FLAG_RXNE | USART_FLAG_TC | USART_FLAG_TXE);
	USART_DMACmd(OW_USART, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
	DMA_Cmd(OW_DMA_CH_RX, ENABLE);
	DMA_Cmd(OW_DMA_CH_TX, ENABLE);

	// ����, ���� �� ������ 8 ����
	while (DMA_GetFlagStatus(OW_DMA_FLAG) == RESET) {
#ifdef OW_GIVE_TICK_RTOS
		taskYIELD();
#endif
	}

	// ��������� DMA
	DMA_Cmd(OW_DMA_CH_TX, DISABLE);
	DMA_Cmd(OW_DMA_CH_RX, DISABLE);
	USART_DMACmd(OW_USART, USART_DMAReq_Tx | USART_DMAReq_Rx, DISABLE);

}

/**
 * @brief	������������ ������������ ���� 1-wire � ���������� ���������
 * 			ID ��������� � ������ buf, �� 8 ���� �� ������ ����������.
 * @param	buf - ��� ���������� ID ��������� �� 8 ���� �� ������ ����������
 * @param	num - ������������ ���������� ��������� ���������, ����� �� ����������� �����
 * @retval	����� ��������� ���������
 */
unsigned char ow_scan(unsigned char *buf, unsigned char num) {

	unsigned char found = 0;
	unsigned char *lastDevice;
	unsigned char *curDevice = buf;
	unsigned char numBit, lastCollision, currentCollision, currentSelection;

	lastCollision = 0;
	while (found < num) {
		numBit = 1;
		currentCollision = 0;

		// �������� ������� �� ����� ���������
		ow_send(OW_SEND_RESET, "\xf0", 1, 0, 0, OW_NO_READ);

		for (numBit = 1; numBit <= 64; numBit++) {
			// ������ ��� ����. �������� � ���������������
			ow_tobits(OW_READ_SLOT, ow_buf);
			ow_send_bits(2);

			if (ow_buf[0] == OW_R_1) {
				if (ow_buf[1] == OW_R_1) {
					// ��� �������, ���-�� ���������� � ����������� �����
					return found;
				} else {
					// 10 - �� ������ ����� ������ 1
					currentSelection = 1;
				}
			} else {
				if (ow_buf[1] == OW_R_1) {
					// 01 - �� ������ ����� ������ 0
					currentSelection = 0;
				} else {
					// 00 - ��������
					if (numBit < lastCollision) {
						// ���� �� ������, �� ����� �� ��������
						if (lastDevice[(numBit - 1) >> 3]
								& 1 << ((numBit - 1) & 0x07)) {
							// (numBit-1)>>3 - ����� �����
							// (numBit-1)&0x07 - ����� ���� � �����
							currentSelection = 1;

							// ���� ����� �� ������ �����, ���������� ����� ����
							if (currentCollision < numBit) {
								currentCollision = numBit;
							}
						} else {
							currentSelection = 0;
						}
					} else {
						if (numBit == lastCollision) {
							currentSelection = 0;
						} else {
							// ���� �� ������ �����
							currentSelection = 1;

							// ���� ����� �� ������ �����, ���������� ����� ����
							if (currentCollision < numBit) {
								currentCollision = numBit;
							}
						}
					}
				}
			}

			if (currentSelection == 1) {
				curDevice[(numBit - 1) >> 3] |= 1 << ((numBit - 1) & 0x07);
				ow_tobits(0x01, ow_buf);
			} else {
				curDevice[(numBit - 1) >> 3] &= ~(1 << ((numBit - 1) & 0x07));
				ow_tobits(0x00, ow_buf);
			}
			ow_send_bits(1);
		}
		found++;
		lastDevice = curDevice;
		curDevice += 8;
		if (currentCollision == 0)
			return found;

		lastCollision = currentCollision;
	}

	return found;
}
