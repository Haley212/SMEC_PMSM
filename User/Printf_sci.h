
#ifndef PRINTF_SCI_H_
#define PRINTF_SCI_H_


#define SCIDATABUFF 1024

extern volatile unsigned int SciDataOutP,SciDataInP;
extern char SciDataBuff[SCIDATABUFF];

void sPrintf(char *f, ...);



#endif


