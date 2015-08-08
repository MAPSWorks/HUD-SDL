[1mdiff --git a/common/gui.c b/common/gui.c[m
[1mindex 435bb18..52462c6 100644[m
[1m--- a/common/gui.c[m
[1m+++ b/common/gui.c[m
[36m@@ -8,8 +8,10 @@[m
 #include <iostream>[m
 #include <vector>[m
 #include "common.h"[m
[32m+[m[32m#include "bluetooth_top.h"[m
 [m
[31m-extern char sensors_buf[BUF_SIZE], bt_buf[BUF_SIZE], gps_buf[BUF_SIZE] ,velocity_buf[BUF_SIZE];[m
[32m+[m[32mextern char sensors_buf[BUF_SIZE], gps_buf[BUF_SIZE];[m
[32m+[m[32mextern BT_data bt_data;[m
 extern VnDeviceCompositeData sensorData;[m
 extern bool globQuitSig;[m
 int velocity = 0;[m
[36m@@ -163,11 +165,8 @@[m [mbool loadMedia()[m
 [m
 bool reloadText()[m
 {[m
[31m-	int velocityInt = velocity;[m
[31m-	int gearInt = gear;[m
[31m-[m
[31m-	std::string strVelocity = std::to_string(velocityInt);[m
[31m-	std::string strGear = std::to_string(gearInt);[m
[32m+[m	[32mstd::string strVelocity = std::to_string(velocity);[m
[32m+[m	[32mstd::string strGear = std::to_string(gear);[m
 [m
 	//Render text[m
 	return 	gTextVelocity.loadFromRenderedText(strVelocity, VEL_FONT_COLOR,gDigitalFont) &&[m
[36m@@ -534,7 +533,6 @@[m [mvoid* gui_main(void* arg)[m
 				RPMint      %= MAX_RPM;[m
 				gearInt     %= MAX_GEAR;[m
 				if(gearInt==0) gearInt=1;[m
[31m-				degrees     = (velocityInt*MAX_DEGREE)/MAX_VELOCITY + DEGREES_OFFSET;[m
 [m
 				// connecting with the global variables[m
 				RPM = RPMint;[m
[36m@@ -549,7 +547,13 @@[m [mvoid* gui_main(void* arg)[m
 #endif[m
 [m
 				horDeg = (double)sensorData.ypr.pitch;[m
[31m-[m
[32m+[m				[32mRPMint = (int)bt_data.rpm;[m
[32m+[m				[32mvelocityInt = (int)bt_data.velo;[m
[32m+[m				[32mvelocity = velocityInt;[m
[32m+[m				[32mdegrees     = (velocityInt*MAX_DEGREE)/MAX_VELOCITY + DEGREES_OFFSET;[m
[32m+[m				[32mgearInt = bt_data.gear;[m
[32m+[m				[32mgear = gearInt;[m
[32m+[m				[32mstrGear = std::to_string(gearInt);[m
 [m
 				//Clear screen[m
 				SDL_SetRenderDrawColor( gRenderer, 0, 0, 0, 0 );	//	background screen color[m
[36m@@ -645,8 +649,8 @@[m [mvoid* gui_main(void* arg)[m
                         angRot = utils.CoordinateAngle(prevCo,currCo);[m
                     else[m
                         angRot = 0;[m
[31m-                    printf("prevCo=(%f,%f),currCo = (%f,%f)\n",prevCo.X,prevCo.Y,currCo.X,currCo.Y);[m
[31m-                    printf("angleRot = %f\n",angRot);[m
[32m+[m[32m                    //printf("prevCo=(%f,%f),currCo = (%f,%f)\n",prevCo.X,prevCo.Y,currCo.X,currCo.Y);[m
[32m+[m[32m                    //printf("angleRot = %f\n",angRot);[m
                     //Updating map stuff[m
                     //printf("test5\n");[m
                     utils.UpdateMap(originalPts,mapPts,vecVelocity,origin, deltaXY,newPointSampled,angRot);[m
