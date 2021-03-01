/* 
 * File:   Feature3DEngine.h
 * Author: arwillis
 *
 * Created on August 18, 2015, 10:35 AM
 */

#ifndef CALIBRATE_MOCAP_AND_CAMERA_H
#define CALIBRATE_MOCAP_AND_CAMERA_H

// Standard C++ includes
#include <string>
#include <iostream>
#include <fstream>
#include <math.h>

// ROS includes
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

class CalibrateMocapAndCamera {
public:
    typedef boost::shared_ptr<CalibrateMocapAndCamera> Ptr;

    CalibrateMocapAndCamera(std::string optical_parent, std::string optical_frame,
    bool _logdata, std::string _logfilename) :
    map_frame_id_str(optical_parent),
    rgb_frame_id_str(optical_frame),
    logdata(_logdata),
    logfilename(_logfilename) {
        if (_logdata) {
            std::cout << "Opening logfile " << _logfilename << "." << std::endl;
            fos.open(_logfilename.c_str());
        }
    }

    virtual ~CalibrateMocapAndCamera() {
        if (logdata) {
            fos.close();
        }
    }

    void initializeSubscribers(ros::NodeHandlePtr nodeptr,
            std::string tf_camera_topic, std::string tf_calib_topic, std::string ar_calib_topic, int timeval = 10) {
      sub_tf_aruco_calib_pose = nodeptr->subscribe(ar_calib_topic, 1, &CalibrateMocapAndCamera::ar_calib_pose_Callback, this);
    }
    void ar_calib_pose_Callback(const geometry_msgs::TransformStampedConstPtr& ar_calib_pose);


private:
    // -------------------------
    // Disabling default copy constructor and default
    // assignment operator.
    // -------------------------
    CalibrateMocapAndCamera(const CalibrateMocapAndCamera& yRef);
    CalibrateMocapAndCamera& operator=(const CalibrateMocapAndCamera& yRef);


    // variable used when using a ground truth reference initialization
    ros::Subscriber sub_tf_aruco_calib_pose;

    std::string map_frame_id_str;
    std::string rgb_frame_id_str;

    bool logdata;
    std::string logfilename;
    std::ofstream fos;
};
#endif /* CALIBRATE_MOCAP_AND_CAMERA_H */

