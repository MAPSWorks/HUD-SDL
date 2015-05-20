#include <stdio.h>
#include <unistd.h>
#include "vectornav.h"
#include "sensorenv.h"
#include "bluetooth_top.h"

// This is a test comment
int main()
{
	sensors_main();
	BT_main();
	return 0;
}
