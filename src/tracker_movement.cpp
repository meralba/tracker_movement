/*
* Copyright (c) 2013, Marcus Liebhardt, Yujin Robot.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Yujin Robot nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

/*
 * Inspired by the openni_tracker by Tim Field and PrimeSense's NiTE 2.0 - Simple Skeleton Sample
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>




//#include <OpenNI.h>
//#include <XnCodecIDs.h>
//#include <XnCppWrapper.h>
#include <NiTE.h>
#include <tracker_movement/Viewer.h>
#include <std_msgs/String.h>
using std::string;

//xn::Context        g_Context;
//xn::DepthGenerator g_DepthGenerator;
//xn::UserGenerator  g_UserGenerator;


int main(int argc, char **argv) {
    ros::init(argc, argv, "tracker_movement");

	if(!ros::ok())
  	{
		ROS_ERROR("Couldn't initialize node");
    	exit(1); //TODO cambiar por salida de errores
  	}

	ros::NodeHandle nh, nh_priv("~");
	ros::Publisher gripper_pub = nh.advertise<std_msgs::String>("/RosAria/cmd_gripper", 1000);
	ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1000);
	ros::Publisher ptu_pub = nh.advertise<std_msgs::String>("/RosAria/cmd_ptu", 1000);
    openni::Status statusOpenni;
    nite::Status niteStatus;
    Viewer viewer("tracking",gripper_pub, velocity_pub, ptu_pub);


    statusOpenni = viewer.Init(argc,argv);
	ros::Rate r(10);
    if (statusOpenni != openni::STATUS_OK){
        ROS_ERROR("Couldn't initialize viewer");
    }
	/*while (ros::ok())
	{
		std_msgs::String cmdGripper;

		std::stringstream ss;
		ss << "";
		cmdGripper.data = ss.str();
		niteStatus = viewer.Run(cmdGripper);
		if(niteStatus != nite::STATUS_OK){
			cerr << "Something is worng in function run..." << endl;
			exit(3);
		}
		if(cmdGripper.data != ""){
			ROS_INFO_STREAM("Send to /RosAria/cmd_gripper the value " << cmdGripper.data);
			gripper_pub.publish(cmdGripper);
		}
		r.sleep();
	}
    */

    viewer.Run();
    printf("HOASDOIASDIOAS\n");
	return 0;

}
