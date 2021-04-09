#!/usr/bin/env python3

import rospy
import sensor_msgs.msg

if __name__ == "__main__":
    rospy.init_node('depth_camera_info_publisher', anonymous=True, log_level=rospy.INFO)
    camera_name = str(rospy.get_param("~camera_name"))
    width = int(rospy.get_param("~width"))
    height = int(rospy.get_param("~height"))

    if width!=848 or height!=480:
        raise NotImplementedError("Currently only 848x480 is supported. Received "+str(width)+"x"+str(height))

    pub = rospy.Publisher("camera_info", sensor_msgs.msg.CameraInfo, queue_size=1) 

    rate = rospy.Rate(60) # 60Hz    
    while not rospy.is_shutdown():
        msg = sensor_msgs.msg.CameraInfo()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = camera_name+"_depth_optical_frame"
        msg.height = 480
        msg.width = 848
        msg.distortion_model = 'plumb_bob'
        msg.D = [0,0,0,0,0]
        msg.K = [427.0580749511719, 0.0, 427.1028747558594, 0.0, 427.0580749511719, 237.02572631835938, 0.0, 0.0, 1.0]
        msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.P = [427.0580749511719, 0.0, 427.1028747558594, 0.0, 0.0, 427.0580749511719, 237.02572631835938, 0.0, 0.0, 0.0, 1.0, 0.0]
        msg.binning_x = 0
        msg.binning_y = 0
        msg.roi.x_offset = 0
        msg.roi.y_offset = 0
        msg.roi.height = 0
        msg.roi.width = 0
        msg.roi.do_rectify = False

        pub.publish(msg)

        rate.sleep()

