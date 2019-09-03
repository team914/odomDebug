#include "main.h"
#include "odomDebug/odomDebug.hpp"
#include "autolib/api.hpp"
#include "okapi/api.hpp"

autolib::Pose startPose{ 1_ft, 3_ft, 90_deg };
bool reverse = true;
double x = -1;
double y = 1;

autolib::Pose pose = startPose;

void setState(QLength x, QLength y, QAngle theta) {
    // set these values to your odometry
    // to convert the QUnits to doubles, call
    // `x.convert(inch)` or `theta.convert(radian)`
    // you can use any length or angle unit
    pose.x = x;
    pose.y = y;
    pose.yaw = theta;
}

void resetSensors() {
    // reset sensors
    // reset odometry
	pose = startPose;
}

void opcontrol() {
    pros::Controller master( pros::E_CONTROLLER_MASTER );

    okapi::Logger::setDefaultLogger(
        std::make_shared<Logger>(std::make_unique<Timer>(), "/ser/sout", Logger::LogLevel::debug));


	autolib::PathGenerator pathGenerator( { 1.0, 2.0, 4.0 } );
	pathGenerator.generatePath( {
		autolib::Pose{ 1_ft, 1_ft, 180_deg }
	}, std::string("test")
	);

	auto paths = pathGenerator.getPaths();

    OdomDebug display(lv_scr_act(), LV_COLOR_ORANGE);
    display.setStateCallback(setState);
    display.setResetCallback(resetSensors);

    for( auto &&path: paths ){

        // set your odometry data here (position and sensor data)
        // you can use QUnits for the x, y, and theta
        // or you can use doubles, in inches and radians
        // the last `middle` paramiter is optional, depending on your robot
        // display.setData(x, y, theta, left, right, middle);

        //display.setData({0, 0, 0}, {0, 0});
		for( auto &&ipose: path.path ){
			if(reverse)
				pose = autolib::Pose{ (y * ipose.pose.x) * meter, (x * ipose.pose.y) * meter, (ipose.pose.x * radian) - startPose.yaw  };
			else
				pose = autolib::Pose{ (x * ipose.pose.x) * meter, (y * ipose.pose.y) * meter, (ipose.pose.x * radian) - startPose.yaw  };
			display.setData({pose.x, pose.y, pose.yaw}, {0, 0, 0});
			pros::delay(100);
		}
    }
}
