/**
 * @file	ds18b20.h
 * @brief	Драйвер температурного датчика ds18b20
 */

#ifndef _ds18b20_h_
#define _ds18b20_h_

extern const char
	*t_frac[16];
extern unsigned char
	ds_flag,
	id_count,
	bid[32],
	buf[10];

unsigned char ds18b20_init();
void ds18b20_start();
void ds18b20_result();
void ds18b20_get_temp(unsigned char *sign, unsigned short *temp, unsigned char cnt);

#endif
