/*
 * File:   Feature3DEngine.cpp
 * Author: arwillis
 *
 * Created on August 18, 2015, 10:35 AM
 */
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <iterator>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>

// Eigen includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

// TF includes
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// Includes for this Library
#include <calibrate_mocap_and_camera/calibrate_mocap_and_camera.h>

#include <eigen_conversions/eigen_msg.h>

bool validTransform(tf::StampedTransform & transform) {
	bool result = true;
	double normTranslation = transform.getOrigin().x() *transform.getOrigin().x() + transform.getOrigin().y() *transform.getOrigin().y() +transform.getOrigin().z() *transform.getOrigin().z() ;
	double normQuaternion = transform.getRotation().x()*transform.getRotation().x() + transform.getRotation().y()*transform.getRotation().y() + transform.getRotation().z()*transform.getRotation().z() + transform.getRotation().w()*transform.getRotation().w();
	if (( fabs(sqrt(normQuaternion)) > 1 ) or (sqrt(normTranslation) > 100000))
		{
		result = false;
		}
	return result;
		
    }

void CalibrateMocapAndCamera::ar_calib_pose_Callback(const geometry_msgs::TransformStampedConstPtr& ar_calib_pose) {
    static tf::TransformListener listener;
    static tf::TransformBroadcaster br;
    static geometry_msgs::TransformStampedConstPtr prior_tf;

    std::string cam_frame_id_str("tf_cam");
    std::string calib_frame_id_str("tf_calib");
    tf::StampedTransform cam_marker_pose;
    tf::StampedTransform calib_marker_pose;
    tf::StampedTransform tf_cam_to_rgb_optical_frame;
    static bool DEBUG = false;
    if (DEBUG) {
        ROS_INFO("Looking up transform from frame '%s' to frame '%s'", map_frame_id_str.c_str(),
                cam_frame_id_str.c_str());
    }
    ros::Time queryTime = ros::Time::now();
    try {
        listener.waitForTransform(map_frame_id_str, cam_frame_id_str,
                queryTime, ros::Duration(1));
        listener.lookupTransform(map_frame_id_str, cam_frame_id_str,
                queryTime, cam_marker_pose);
        listener.waitForTransform(map_frame_id_str, calib_frame_id_str,
                queryTime, ros::Duration(1));
        listener.lookupTransform(map_frame_id_str, calib_frame_id_str,
                queryTime, calib_marker_pose);
    } catch (tf::TransformException ex) {
        //ROS_ERROR("%s", ex.what());
        //        ros::Duration(1.0).sleep();
    }
    if (DEBUG) {
        std::cout << "cam_pose = " << cam_marker_pose.getOrigin().x() << ", " <<
                cam_marker_pose.getOrigin().y() << ", " << cam_marker_pose.getOrigin().z() << std::endl;
        std::cout << "calib_pose = " << calib_marker_pose.getOrigin().x() << ", " <<
                calib_marker_pose.getOrigin().y() << ", " << calib_marker_pose.getOrigin().z() << std::endl;
        std::cout << "ar_pose = " << ar_calib_pose->transform.translation << std::endl;
    }
    static geometry_msgs::TransformStamped ar_calib_to_camera;

    tf::Transform transform;

    transform.setOrigin(tf::Vector3(ar_calib_pose->transform.translation.x,
            ar_calib_pose->transform.translation.y, ar_calib_pose->transform.translation.z));
    transform.setRotation(tf::Quaternion(ar_calib_pose->transform.rotation.x, ar_calib_pose->transform.rotation.y,
            ar_calib_pose->transform.rotation.z, ar_calib_pose->transform.rotation.w).normalize());
    tf::Transform itransform = transform.inverse();
    br.sendTransform(tf::StampedTransform(itransform,
            ros::Time::now(), "ar_optical_frame", "rgb_optical_pose"));
    try {
        listener.waitForTransform(cam_frame_id_str, "rgb_optical_pose",
                queryTime, ros::Duration(1));
        listener.lookupTransform(cam_frame_id_str, "rgb_optical_pose",
                queryTime, tf_cam_to_rgb_optical_frame);
    } catch (tf::TransformException ex) {
    }
	if(validTransform(tf_cam_to_rgb_optical_frame))
		{
		    br.sendTransform(tf::StampedTransform(tf_cam_to_rgb_optical_frame,
			    ros::Time::now(), "tf_cam", "calib_rgb_optical_pose"));
		}

    if (DEBUG) {
        std::cout << "calib_result = " << tf_cam_to_rgb_optical_frame.getOrigin().x() << ", " <<
                tf_cam_to_rgb_optical_frame.getOrigin().y() << ", " << tf_cam_to_rgb_optical_frame.getOrigin().z() << std::endl;
    }
    tf::Quaternion calib_rot = tf_cam_to_rgb_optical_frame.getRotation();
    tf::Vector3 calib_translation = tf_cam_to_rgb_optical_frame.getOrigin();
    prior_tf = ar_calib_pose;

    std::cout << "calib_tran = [" << calib_translation.getX() << " "
            << calib_translation.getY() << " "
            << calib_translation.getZ() << "] "
            << "calib_quat = ["
            << calib_rot.getX() << " "
            << calib_rot.getY() << " "
            << calib_rot.getZ() << " "
            << calib_rot.getW() << "]" << std::endl;
    if (logdata) {
        fos << calib_translation.getX() << " "
                << calib_translation.getY() << " "
                << calib_translation.getZ() << " "
                << calib_rot.getX() << " "
                << calib_rot.getY() << " "
                << calib_rot.getZ() << " "
                << calib_rot.getW() << " "
                << cam_marker_pose.getOrigin().x() << " "
                << cam_marker_pose.getOrigin().y() << " "
                << cam_marker_pose.getOrigin().z() << " "
                << cam_marker_pose.getRotation().x() << " "
                << cam_marker_pose.getRotation().y() << " "
                << cam_marker_pose.getRotation().z() << " "
                << cam_marker_pose.getRotation().w() << " "
                << calib_marker_pose.getOrigin().x() << " "
                << calib_marker_pose.getOrigin().y() << " "
                << calib_marker_pose.getOrigin().z() << " "
                << calib_marker_pose.getRotation().x() << " "
                << calib_marker_pose.getRotation().y() << " "
                << calib_marker_pose.getRotation().z() << " "
                << calib_marker_pose.getRotation().w() <<std::endl;
    }
}

int main(int argc, char **argv) {
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "calibrate_mocap_and_camera");
    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    std::string tf_camera_marker_topic, tf_calib_marker_topic, ar_calib_topic;
    std::string optical_parent, optical_frame, _logfilename;
    bool _logdata;
    ros::NodeHandlePtr nodeptr(new ros::NodeHandle);
    ros::NodeHandle privnh("~");

    privnh.param<std::string>("tf_cam_topic", tf_camera_marker_topic, "/tf_cam/pose");
    privnh.param<std::string>("tf_calib_topic", tf_calib_marker_topic, "/tf_calib/pose");
    privnh.param<std::string>("ar_calib_topic", ar_calib_topic, "/ar_single_board/transform");
    privnh.param("logdata", _logdata, false);
    privnh.param<std::string>("logfilename", _logfilename, "");
    privnh.param<std::string>("optical_parent", optical_parent, "optitrack");
    privnh.param<std::string>("optical_frame", optical_frame, "rgb_optical_frame");
    std::cout << "RGBD parent coordinate frame name = \"" << optical_parent << "\"" << std::endl;
    std::cout << "RGBD coordinate frame name =  \"" << optical_frame << "\"" << std::endl;
    CalibrateMocapAndCamera::Ptr engineptr(new CalibrateMocapAndCamera(optical_parent,
            optical_frame, _logdata, _logfilename));

    engineptr->initializeSubscribers(nodeptr, tf_camera_marker_topic, tf_calib_marker_topic,
            ar_calib_topic);
    ros::spin();
    return 0;
}


