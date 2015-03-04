/*
 * Menu.h
 *
 *  Created on: 2014-7-1
 *      Author: appleseed
 */

#ifndef MENU_H_
#define MENU_H_

extern volatile unsigned char MachineState;
extern volatile unsigned char MotorState;
extern unsigned char SCICmdBuffer[8];
extern unsigned char SCICmdBufferIndex;

#define MAX_CMD 24

typedef struct MenuCmd{
	char *cmd;
	void (*fp)();
}MenuCmd;

typedef struct MenuStruct{
	MenuCmd CMD[MAX_CMD];
}MenuStruct;

void MainMenu(void);

#endif /* MENU_H_ */
