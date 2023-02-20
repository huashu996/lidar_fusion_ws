#include "multi_lidar_calibrator.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, __APP_NAME__);

	ROSMultiLidarCalibratorApp app;

	app.Run();

	return 0;
}
