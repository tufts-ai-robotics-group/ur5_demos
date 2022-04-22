#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <std_msgs/String.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


#include <bimur_robot_vision/TabletopPerception.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

// moveit
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/GetPositionIK.h>
#include <robotiq_85_msgs/GripperCmd.h>




/*
  joint_states of a pose out of the way
  name: [elbow_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint, wrist_2_joint,
  wrist_3_joint]
  position: [1.7994565963745117, -1.304173771535055, 0.46854838728904724, -2.487392250691549, -4.015228335057394, -0.6358125845538538]
  velocity: [-0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  effort: [1.1388397216796875, 2.6812055110931396, 1.275590181350708, 0.4529372453689575, 0.16012932360172272, 0.06405173242092133]

 */

ros::Publisher pose_pub;
ros::Publisher gripper_pub;

ros::ServiceClient ik_client;

geometry_msgs::PoseStamped current_button_pose;
moveit::planning_interface::MoveGroupInterface *group;

float Z_OFFSET = 0.25;
float NEW_Z_OFFSET = 0.0;

bool g_caught_sigint = false;

void sig_handler(int sig){
    g_caught_sigint = true;
    ROS_INFO("caugt sigint, init shutdown seq...");
    ros::shutdown();
    exit(1);
}

// Blocking call for user input
void pressEnter(){
	std::cout << "Press the ENTER key to continue";
	while (std::cin.get() != '\n')
		std::cout << "Please press ENTER\n";
}



bool close_gripper(){
	
	

	robotiq_85_msgs::GripperCmd msg;
	msg.position = 0.0;
	msg.speed = 1.0;
	msg.force = 100.0;
	gripper_pub.publish(msg);

	ros::spinOnce();
	
	
	return 0;
}

bool open_gripper(){
	
	robotiq_85_msgs::GripperCmd msg;
	msg.position = 1.0;
	msg.speed = 1.0;
	msg.force = 100.0;
	gripper_pub.publish(msg);

	ros::spinOnce();
	
	
	return 0;

}


void detectObjects(ros::NodeHandle n){
	//step 1: call the button detection service and get the response
	ros::ServiceClient client = n.serviceClient<bimur_robot_vision::TabletopPerception>("/bimur_object_detector/detect");
	bimur_robot_vision::TabletopPerception srv;
	
	ROS_INFO("Detecting objects.");
	
	if(client.call(srv)) {
		
		ROS_INFO("Service called.");
		
		if(srv.response.is_plane_found == false)
			ros::shutdown();
		
		std::vector<pcl::PointCloud<pcl::PointXYZ>> detected_objects;
		
		int num_objects = srv.response.cloud_clusters.size();
		ROS_INFO("num_objects: %i", num_objects);
		
		for (int i = 0; i < srv.response.cloud_clusters.size(); i++)
		{
			//convert each object to pcl format
			pcl::PointCloud<pcl::PointXYZ> cloud_i;
			pcl::fromROSMsg(srv.response.cloud_clusters[i], cloud_i);
			
			//store it
			detected_objects.push_back(cloud_i);
		}
		
		//decide which object to interact with
		//TODO: find largest object
		int target_object_index = 0;
		int max = detected_objects[0].points.size();
		for (int i = 0; i < detected_objects.size(); i++) {
			int num_points = detected_objects[i].points.size();
			if (num_points > max) {
				max = num_points;
				target_object_index = i;
			}
		}
		
		ROS_INFO("Found max");
		
		//pick target object
		pcl::PointCloud<pcl::PointXYZ> target_object = detected_objects[target_object_index];
		
		//create a pose that is above the object by 10 cm
		
			
		//step 2. convert the cloud in the response to PCL format
		//sensor_msgs::PointCloud2 input = srv.response.cloud_button;
		//pcl::fromROSMsg(input, pcl_cloud);
		
		//step 3. create a pose with x y z set to the center of point cloud

		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(target_object, centroid);
		
		ROS_INFO("Centroid x: %f", centroid(0));
		ROS_INFO("Centroid y: %f", centroid(1));
		ROS_INFO("Centroid z: %f", centroid(2));

		//transforms the pose into /map frame
		geometry_msgs::Pose pose_i;
		pose_i.position.x=centroid(0);
		pose_i.position.y=centroid(1);
		pose_i.position.z=centroid(2);
		pose_i.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);

		geometry_msgs::PoseStamped stampedPose;

		stampedPose.header.frame_id = target_object.header.frame_id;
		stampedPose.header.stamp = ros::Time(0);
		stampedPose.pose = pose_i;

		tf::TransformListener listener; 

		//step 4. transform the pose into Mico API origin frame of reference
		geometry_msgs::PoseStamped stampOut;
		listener.waitForTransform(target_object.header.frame_id, "world", ros::Time(0), ros::Duration(3.0));
		listener.transformPose("world", stampedPose, stampOut);

		ROS_INFO("[move_arm] publishing pose...");

		//stampOut.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,-3.14/2,0);
		
		//roll rotates around x axis
		
		stampOut.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14/2, 3.14/2,0.0);
		stampOut.pose.position.z+= Z_OFFSET;
		//stampOut.pose.position.x+=0.09;
		
		ROS_INFO_STREAM(stampOut);
		
		pose_pub.publish(stampOut);
		
		//store pose
		current_button_pose = stampOut;
	
		ros::spinOnce();
	
		//step 4.5. adjust post xyz and/or orientation so that the pose is above the button and oriented correctly
	
		//step 5. publish the pose	
	}
}

void moveAboveObject() {
	
	ROS_INFO("Starting to move above object");
		
	group->setPoseReferenceFrame(current_button_pose.header.frame_id);
	group->setPoseTarget(current_button_pose);
	
	
    group->setStartState(*group->getCurrentState());
    
    moveit_msgs::GetPositionIK::Request ik_request;
    moveit_msgs::GetPositionIK::Response ik_response;
    ik_request.ik_request.group_name = "manipulator";
    ik_request.ik_request.pose_stamped = current_button_pose;
    
    ik_client.call(ik_request, ik_response);
    
    ROS_INFO_STREAM(ik_response);
    
    ROS_INFO("Starting to plan...");
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success = group->plan(my_plan);
    ROS_INFO("Planning success: %s", success ? "true" : "false");

    if (!success) {
        return;
    }
    
    pressEnter();
    
    moveit::planning_interface::MoveItErrorCode error = group->move();

}


void moveToObj(){
	

	//Reduce the offset created when calling DetectObjects() by lowering frame 
	current_button_pose.pose.position.z -= 0.10;
	
	
	group->setPoseReferenceFrame(current_button_pose.header.frame_id);
	group->setPoseTarget(current_button_pose);
	
	
    group->setStartState(*group->getCurrentState());
    
    moveit_msgs::GetPositionIK::Request ik_request;
    moveit_msgs::GetPositionIK::Response ik_response;
    ik_request.ik_request.group_name = "manipulator";
    ik_request.ik_request.pose_stamped = current_button_pose;
    
    ik_client.call(ik_request, ik_response);
    
    ROS_INFO_STREAM(ik_response);
    
    ROS_INFO("Starting to plan...");
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success = group->plan(my_plan);
    ROS_INFO("Planning success: %s", success ? "true" : "false");

    if (!success) {
        return;
    }
    
    pressEnter();
    
	ROS_INFO("Moving down...");
    moveit::planning_interface::MoveItErrorCode error = group->move();
    current_button_pose.pose.position.z += 0.10;
	
}

void move_to_pos(){
	
    
	group->setPoseReferenceFrame(current_button_pose.header.frame_id);
	group->setPoseTarget(pose);
	
	
    group->setStartState(*group->getCurrentState());
    
    moveit_msgs::GetPositionIK::Request ik_request;
    moveit_msgs::GetPositionIK::Response ik_response;
    ik_request.ik_request.group_name = "manipulator";
    ik_request.ik_request.pose_stamped = pose;
    
    ik_client.call(ik_request, ik_response);
    
    ROS_INFO_STREAM(ik_response);
    
    ROS_INFO("Starting to plan...");
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success = group->plan(my_plan);
    ROS_INFO("Planning success: %s", success ? "true" : "false");

    if (!success) {
        return;
    }
    
    pressEnter();
    
	ROS_INFO("Moving down...");
    moveit::planning_interface::MoveItErrorCode error = group->move();

}

int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "move_arm");

	ros::NodeHandle n;
	ros::AsyncSpinner spinner(1);
    spinner.start();
    
	ik_client = n.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
	
	//button position publisher
	pose_pub = n.advertise<geometry_msgs::PoseStamped>("/move_arm_demo/pose", 10);
	
	//gripper command publisher
	gripper_pub = n.advertise<robotiq_85_msgs::GripperCmd>("/gripper/cmd",10);
	
	
	ROS_INFO("Demo starting...Move the arm to a 'ready' position that does not include the table.");
	pressEnter();
	open_gripper();
	
	
	
	detectObjects(n);
	
	pressEnter();
	
	group = new moveit::planning_interface::MoveGroupInterface("manipulator");
    group->setGoalTolerance(0.01);
    
    
	moveAboveObject();
	
	pressEnter();
	
    
    moveToObj();
 
    pressEnter();
    
    close_gripper();
    
    moveAboveObject(); 
    
    pressEnter();
    
    move_to_pos();
    
	pressEnter();
    
    

	return 0;
}
