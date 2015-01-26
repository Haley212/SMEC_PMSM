


#include "Printf_sci.h"
#include <stdarg.h>


volatile unsigned int SciDataOutP=0,SciDataInP=0;

char SciDataBuff[SCIDATABUFF]={0};


void Putchar(unsigned char data)
{
	if(((SciDataInP+1)%SCIDATABUFF)!=SciDataOutP)
	{
		SciDataBuff[SciDataInP]=data;
		SciDataInP=(SciDataInP+1)%SCIDATABUFF;
	}
}


void  PutString(char *str)
{
	while(*str !='\0')
	{		
		Putchar(*str++);
		
	}
}


 int strlen(const char *s)
{
	int i = 0;

	for(; *s; s++)
		i++;
	
	return i;
}

void PutRepChar(char c, int count)
{
	while (count--) Putchar(c);
}

/* put string reverse */
void PutStringReverse(char *s, int index)
{
  while ((index--) > 0) Putchar(s[index]);
}



static void PutNumber(int value, int radix, int width, char fill)
{
  char buffer[40];
  int bi = 0;
  int unsigned uvalue;
  short int digit;
  short int left =0;
  short int negative =0;

  if (fill == 0) fill = ' ';

  if (width < 0) {
    width = -width;
    left = 1;
  }
  if (width < 0 || width > 80) width = 0;
  
  if (radix < 0) {
    radix = -radix;
    if (value < 0) {
      negative = 1;
      value = -value;
    }
  }
  uvalue = value;
  do {
    if (radix != 16) {
      digit = uvalue % radix ;
      uvalue = uvalue / radix ;
    }
    else {
      digit = uvalue & 0xf;
      uvalue = uvalue >> 4;
    }
    buffer[bi] = digit + ((digit <= 9) ? '0' : ('A' - 10));
    bi++;

    if (uvalue != 0) {
      if ((radix==10)&&((bi==3)||(bi==7)||(bi==11)|(bi==15))) {
//	buffer[bi++]=',';
      }
    }
  } while (uvalue != 0);

  if (negative) {
    buffer[bi] = '-';
    bi += 1;
  }
  if (width <= bi) PutStringReverse(buffer, bi);
  else {
    width -= bi;
    if (!left) PutRepChar(fill, width);
    PutStringReverse(buffer, bi);
    if (left) PutRepChar(fill, width);
  } 
}
static char *FormatItem(char *f, int a)
{
  char c;
  int fieldwidth = 0;
  int leftjust = 0;
  int radix = 0;
  char fill = ' ';

  if (*f == '0') fill = '0';
  while ((c = *f++)!=0) {
    if (c >= '0' && c <= '9') {
      fieldwidth = (fieldwidth * 10) + (c - '0');
    }
    else switch (c) {
    case '\000': return(--f);
    case '%': Putchar('%');
      return(f);
    case '-': leftjust = 1;
      break;
    case 'c': {
      if (leftjust) Putchar(a & 0x7f);
      if (fieldwidth > 0) PutRepChar(fill, fieldwidth - 1);
      if (!leftjust) Putchar(a & 0x7f);
      return(f);
    }
    case 's': {
      if (leftjust) PutString((char *) a);
      if (fieldwidth > strlen((char *) a))
	PutRepChar(fill, fieldwidth - strlen((char *)a));
      if (!leftjust) PutString((char *) a);
      return(f);
    }
    case 'd': 
    case 'i': radix = -10;break;
    case 'u': radix = 10;break;
    case 'x': radix = 16;break;
    case 'X': radix = 16;break;
    case 'o': radix = 8;break;
    default : radix = 3;break;/* unknown switch! */
    }
    if (radix) break;
  }
  if (leftjust) fieldwidth = -fieldwidth;
  PutNumber(a, radix, fieldwidth, fill);
  return(f);
}

void sPrintf(char *f, ...)
{
	char *argP;
	va_start(argP,f);		/* point at the end of the format string */
	while (*f)
	{			/* this works because args are all ints */
		if (*f == '%')
			f = FormatItem(f + 1, va_arg(argP, int));
		else
			Putchar(*f++);
	}
	va_end(argP);
}




