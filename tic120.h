#ifndef _TIC120_H_
#define _TIC120_H_

void tic120_init();
void tic120_clear();
void tic120_set_pos(unsigned char x, unsigned char y);
void tic120_write_char(char ch);
void tic120_write_char_pos(char ch, unsigned char x, unsigned char y);
void tic120_write_string(char *str);
void tic120_write_string_pos(char *str, unsigned char x, unsigned char y);
void tic120_printf(char *str, unsigned char x, unsigned char y, ...);

#endif
