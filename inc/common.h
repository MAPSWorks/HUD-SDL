#ifndef COMMON_H_
#define COMMON_H_

#define BT_REFRESH_RATE 1000000
#define SENS_REFRESH_RATE 10000
#define BUF_SIZE 2048	//	it appears a minimum of 1024 bytes buffer size is required, 2048 is taken for stability

#ifdef 	ALEX_COMP
#define PROJ_HOME "/home/alex/Workspace/hud_proj/"
#endif
#ifndef PROJ_HOME
#define PROJ_HOME "/home/odroid/HUD_proj/"
#endif

#endif

