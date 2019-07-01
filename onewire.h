/**
 * @file	onewire.h
 * @brief	Драйвер интерфейса 1-wire
 */

#ifndef ONEWIRE_H_
#define ONEWIRE_H_

// для разных процессоров потребуется проверить функцию ow_Init
// на предмет расположения ножек USART
#include "stm32f10x.h"

// выбираем, на каком USART находится 1-wire
#define OW_USART1
//#define OW_USART2
//#define OW_USART3
//#define OW_USART4


// если нужно отдавать тики FreeRTOS, то раскомментировать
//#define OW_GIVE_TICK_RTOS

// первый параметр функции OW_Send
enum {
	OW_SEND_RESET	= 1,
	OW_NO_RESET		= 2
};

#define NULL	0

// статус возврата функций
enum {
	OW_OK			= 1,
	OW_ERROR		= 2,
	OW_NO_DEVICE	= 3
};

#define OW_NO_READ		0xff
#define OW_READ_SLOT	0xff

void ow_init(void);
unsigned char ow_reset(void);
unsigned char ow_send(unsigned char sendReset, char *command, unsigned char cLen, unsigned char *data, unsigned char dLen, unsigned char readStart);
unsigned char ow_scan(unsigned char *buf, unsigned char num);
void ow_out_set_as_power_pin(void);
void ow_out_set_as_tx_pin(void);

#endif /* ONEWIRE_H_ */
