#ifndef VOFA_H
#define VOFA_H
#include "struct_typedef.h"
#include <stdio.h>
#include "stdarg.h"
typedef union Resolve
{
	float 		float_data;
	uint8_t 	char_table[4];
}Resolve_Typedef;

int fputc(int ch, FILE *f);

void vofa_send(void);
void transmit_to_VOFA(void);

#endif 
