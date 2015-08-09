#ifndef COMMON_H_
#define COMMON_H_

#define BT_REFRESH_RATE 1000000
#define SENS_REFRESH_RATE 10000
#define BUF_SIZE 64

#ifdef 	ALEX_COMP
#define PROJ_HOME "/home/alex/Workspace/hud_proj/"
#elif	MAN_HOME
//	to be passed as an argument to the compiler
#else
#define PROJ_HOME "/home/odroid/project/"
#endif

#endif

