
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Mavlink.h>
#include <geometry_msgs/Quaternion.h>
#include "tf/transform_datatypes.h"

//#include <ugvc_navigation/flight_command.h>



mavros_msgs::State current_state;

//ugvc_navigation::flight_command current_obstacle;

float height_cmd;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped quat_body;
void quat_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    quat_body = *msg;
}


//void obstacle_cb(const ugvc_navigation::flight_command::ConstPtr& msg)
//{
//	current_obstacle = *msg;
//}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_uav_node");
    ros::NodeHandle nh;
    
    double yaw_desired, pitch_desired, roll_desired;
    double vx_desried;


    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Publisher uav_att_pub = nh.advertise<geometry_msgs::Quaternion>
            ("mavros/setpoint_raw/target_attitude",10);
    ros::Subscriber quat_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10, quat_cb);     
	//ros::Subscriber obstacle_sub = nh.subscribe<ugvc_navigation::flight_command>("current_obstacle_status",10,obstacle_cb);                          
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);//caosu 20

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;//*sin(ros::Time::now().toSec()/3.1415);
    pose.pose.position.y = 0;//*cos(ros::Time::now().toSec()/3.1415);
    pose.pose.position.z = 1.5;//给一个开始的高度期望，防止无人机拉不起来

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
//	ROS_INFO("sending");
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok())
	{

		/*------------------------------利用差值来控制无人机的高度------------------------------------*/
	//	if(current_height.range > 1.0) // take off detected

	//	{
	//	  double desired_height = 1.0;
	//	  double delta_z = desired_height - current_height.range; //两者均为tfmini的数据

		  //基于当前的local_z来计算给定的高度
		  //范围在-0.1 ~ 0.1之间的话，是不考虑调整高度的
		  //P控制器
		  //double Param_p = 0.005;
		  //pose.pose.position.z += Param_p*delta_z;	
	//	}
		//得到这个高度差（实际减去期望）
		

		ROS_INFO("Desired z: %f", pose.pose.position.z);



		/*--------------------------------设置无人机期望的航向------------------------------------*/
		//pose.pose.orientation.w = 0.9835;
		pose.pose.orientation.w = 0;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		//pose.pose.orientation.x = 0;
		//pose.pose.orientation.y = 0;
		pose.pose.orientation.z = -1;
		//pose.pose.orientation.z = -0.179;

		tf::Quaternion quat;
		//tf::quaternionMsgToTF(quat_body.pose.orientation, quat);
		tf::quaternionMsgToTF(pose.pose.orientation, quat);
		double roll, pitch, yaw;
		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//transform


		/*-------------------------------基于高度的信息，来调整无人机的位置-----------------------------------*/

		//向右
		if(quat_body.pose.position.z>1.0)//&&turn_right==true)
		{	
//			pose.pose.position.x += 0;// 0.02*sin(yaw)/cos(yaw);  
//			pose.pose.position.y += 0.01;//;+= -0.01*sin(yaw);
		}

		//向左

		if(quat_body.pose.position.z>1.0)//&&turn_left==true)
		{	
	//		pose.pose.position.x += 0; //0.02*sin(yaw)/cos(yaw);  
	//		pose.pose.position.y -= 0.01;
		}

		//Forward
		
        if(quat_body.pose.position.z>1.0)//&&fly_forward==true)
		{
			pose.pose.position.x -= 0.01;
			pose.pose.position.y += 0; //0.02*sin(yaw)/cos(yaw);
		}
		
		//Back emergent when the wall is too near
		{
			//pose.pose.position.x -= 0.005;
			//pose.pose.position.y -= 0.005;
		}


	
			ROS_INFO("yaw: %f", yaw*57.3);

		


        local_pos_pub.publish(pose);

        //uav_att_pub.publish(attitude);
        ROS_INFO("%f %f", pose.pose.position.x, pose.pose.position.y);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
