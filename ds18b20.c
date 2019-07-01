/**
 * @file	ds18b20.c
 * @brief	������� �������������� ������� ds18b20
 */

#include "ds18b20.h"

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "misc.h"

#include "onewire.h"

/**
 * ������ ��� ����������� ������� ����� �����������
 */
const char *t_frac[16]={"0000","0625","1250","1875","2500","3125","3750","4375",
		"5000","5625","6250","6875","7500","8125","8750","9375"};

unsigned char
	ds_flag=0,
	id_count,	// ����� ��������� ��������
	bid[32],	// ����� ��� id ��������
	buf[10];	// ����� ��� �����������

unsigned char ds18b20_init() {
	//debug("\r\r--> 1-wire bus initialization");

	// ������������
	/*RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	// ������� ds18b20 �� ����� PA11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOA->ODR |= GPIO_Pin_11;
	GPIO_Init(GPIOA, &GPIO_InitStructure);*/

	// ������������� USART � DMA
	ow_init();
	// ����� ��������
	id_count=0;
	id_count=ow_scan(bid,4);
	//debug("\rFound %d t%c sensor%c",id_count,0xb0,(((id_count==0)||(id_count>1))?('s'):(' ')));

	return id_count;
}

void ds18b20_start() {
	ow_send(OW_SEND_RESET, "\xcc\x44", 2, NULL, NULL, OW_NO_READ);

    // ��������� ������� ������������ ������ - ������ "�������" �� ����
    ow_out_set_as_power_pin();

	//t_flag=0;
	//fone_timer_init(800, ds18b20_result);
}

void ds18b20_result() {
	//ow_send(OW_SEND_RESET, "\xcc\xbe\xff\xff", 4, buf, 2, 2);

    // ��������������� ������� ����������� UART
    ow_out_set_as_tx_pin();

    // ������ �����������
    uint8_t bf[12];
    bf[0]=0x55;
    bf[9]=0xbe;
    bf[10]=0xff;
    bf[11]=0xff;
    for(int j=0,k=0; j<id_count; j++){
        k=j*8;
    	for(int i=0; i<8; i++){
        	bf[i+1]=bid[i+k];
    	}
        ow_send(OW_SEND_RESET, (char *)bf, 12, &buf[j*2], 2, 10);
    }

    //if((ds_flag++)>=3)
    //	ds_flag=0;
    //ds_flag=1;
}

void ds18b20_get_temp(unsigned char *sign, unsigned short *temp, unsigned char cnt){
	*temp=(buf[1+(cnt*2)]<<8)|buf[0+(cnt*2)];
	*sign=0;
	if(*temp&0x8000){
		*sign=1;
		(*temp)^=0xffff;
		(*temp)++;
	}
}
