//
//	very old shit that should be replaced
//


#include "types.h"


int h_abs(int value)
{
	return value<0 ? -value : value;
}


char* h_itoa(int c)
{
	//convert int to string; avoid using stdlib.h
	static char retString[12];
	char *ptr = retString + 11; //pointer on last element
	int sign = c;
	c = h_abs(c);
	*ptr-- = '\0'; //terminate string
	do
	{
		*ptr-- = c % 10 + '0'; //decrement pointer after assignment
	} while ((c = c/10)); //true as long as c divided by 10 is larger 0
	if (sign < 0) *ptr-- = '-'; //add sign symbol
	return ++ptr;
}



