#!/usr/bin/env python
import os
import thread
import yaml
import rospy
import copy
import numpy as np
import cv2
from duckietown_utils.yaml_wrap import yaml_load_file
import sys

from duckietown import DTROS
from sensor_msgs.msg import CompressedImage, CameraInfo
from image_geometry import PinholeCameraModel
from cv_bridge import CvBridge

class Augmenter(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(Augmenter, self).__init__(node_name=node_name)

        #Get vehicle name
        self.veh_name = str(rospy.get_namespace())
        self.veh_name = str(self.veh_name[1:-1])

        # Load map yaml file
        myargv = rospy.myargv(argv=sys.argv)
        self.mapfile = str(myargv[1])

        try:
            mapfile_dir = ("/code/catkin_ws/src/cra1/packages/augmented_reality/map/{}.yaml".format(self.mapfile))
            self.map = yaml_load_file(mapfile_dir)
            rospy.loginfo("Loaded {}.yaml".format(self.mapfile))
        except:
            rospy.loginfo("ERROR:Unable to find {}.yaml".format(self.mapfile))
            rospy.signal_shutdown("Found no map file ... aborting")

        # Load homography
        self.H = self.load_homography()
        self.Hinv = np.linalg.inv(self.H)
        self.pcm_ = PinholeCameraModel()

        #Load camera parameter
        cam_info = self.load_camera_info()
        self.pcm_.width = cam_info.width
        self.pcm_.height = cam_info.height
        self.pcm_.K = cam_info.K 
        self.pcm_.D = cam_info.D 
        self.pcm_.R = cam_info.R 
        self.pcm_.P = cam_info.P 

        #Load camera resolution
        self.res_w = 640
        self.res_h = 480
        rospy.set_param('/%s/camera_node/exposure_mode' %(self.veh_name) , 'off')
        rospy.set_param('/%s/camera_node/res_w' %(self.veh_name), self.res_w)
        rospy.set_param('/%s/camera_node/res_h' %(self.veh_name), self.res_h)

        #Initialize ROS topics
        self.pub = rospy.Publisher("/{}/augmented_reality_node/{}/image/compressed".format(self.veh_name, self.mapfile), CompressedImage, queue_size=1)
        rospy.loginfo("Publishing augmented reality")
        self.sub_image = rospy.Subscriber("/{}/camera_node/image/compressed".format(self.veh_name), CompressedImage, self.callback, queue_size=1)

    def load_homography(self):
        '''Load homography (extrinsic parameters)'''
        filename = ("/code/catkin_ws/src/cra1/calibrations/camera_extrinsic/"+ self.veh_name + ".yaml")
        
        rospy.loginfo("Using extrinsic calibration of " + self.veh_name)
        data = yaml_load_file(filename)
        rospy.loginfo("Loaded homography for {}".format(self.veh_name))
        return np.array(data['homography']).reshape((3,3))

    def load_camera_info(self):
        '''Load camera intrinsics'''
        filename = ("/code/catkin_ws/src/cra1/calibrations/camera_intrinsic/" + self.veh_name + ".yaml")
        calib_data = yaml_load_file(filename)
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = np.array(calib_data['camera_matrix']['data']).reshape((3,3))
        cam_info.D = np.array(calib_data['distortion_coefficients']['data']).reshape((1,5))
        cam_info.R = np.array(calib_data['rectification_matrix']['data']).reshape((3,3))
        cam_info.P = np.array(calib_data['projection_matrix']['data']).reshape((3,4))
        cam_info.distortion_model = calib_data['distortion_model']
        rospy.loginfo("Loaded camera calibration parameters for {}".format(self.veh_name))
        return cam_info

    def process_image(self, cv_image_raw):
        '''Undistort image'''
        cv_image_rectified = np.zeros(np.shape(cv_image_raw))
        mapx = np.ndarray(shape=(self.pcm_.height, self.pcm_.width, 1), dtype='float32')
        mapy = np.ndarray(shape=(self.pcm_.height, self.pcm_.width, 1), dtype='float32')
        mapx, mapy = cv2.initUndistortRectifyMap(self.pcm_.K, self.pcm_.D, self.pcm_.R, self.pcm_.P, (self.pcm_.width, self.pcm_.height), cv2.CV_32FC1, mapx, mapy)
        return cv2.remap(cv_image_raw, mapx, mapy, cv2.INTER_CUBIC, cv_image_rectified)

    def ground2pixel(self, x,y):
        ground_point = np.array([x, y, 1.0])
        image_point = np.dot(self.Hinv, ground_point)
        image_point = image_point / image_point[2]
        u = image_point[0]
        v = image_point[1]
        return int(u), int(v)

    def draw_segment(self, image, pt_x, pt_y, color):
        defined_colors = {
            'red': ['rgb', [1, 0, 0]],
            'green': ['rgb', [0, 1, 0]],
            'blue': ['rgb', [0, 0, 1]],
            'yellow': ['rgb', [1, 1, 0]],
            'magenta': ['rgb', [1, 0 , 1]],
            'cyan': ['rgb', [0, 1, 1]],
            'white': ['rgb', [1, 1, 1]],
            'black': ['rgb', [0, 0, 0]]}
        _color_type, [r, g, b] = defined_colors[color]
        cv2.line(image, (pt_x[0], pt_y[0]), (pt_x[1], pt_y[1]), (b * 255, g * 255, r * 255), 5)
        return image

    def render_segments(self, index):
        focus_point1 = self.map['segments'][index]['points'][0]
        focus_point2 = self.map['segments'][index]['points'][1]
        color = self.map['segments'][index]['color']

        ref_frame1 = self.map['points'][focus_point1][0]
        if ref_frame1 == 'axle':
            x1, y1, _ = self.map['points'][focus_point1][1]
            x1, y1 = self.ground2pixel(x1, y1)
        if ref_frame1 == 'image01':
            x1, y1 = self.map['points'][focus_point1][1]
            x1, y1 = x1*self.res_w, y1*self.res_h

        ref_frame2 = self.map['points'][focus_point2][0]
        if ref_frame2 == 'axle':
            x2, y2, _ = self.map['points'][focus_point2][1]
            x2, y2 = self.ground2pixel(x2, y2)
        if ref_frame2 == 'image01':
            x2, y2 = self.map['points'][focus_point2][1]
            x2, y2 = x2*self.res_w, y2*self.res_h

        return x1, y1, x2, y2, color

    def callback(self, data):
        #Convert compressed image to BGR
        np_arr = np.fromstring(data.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        img_undistorted = self.process_image(img)

        for index in range(len(self.map['segments'])):
            x1, y1, x2, y2, color = self.render_segments(index)
            img_undistorted = self.draw_segment(img_undistorted, [x1, x2], [y1,y2], color)

        compressed_img = br.cv2_to_compressed_imgmsg(img_undistorted, dst_format='jpg')

        self.pub.publish(compressed_img)

if __name__ == '__main__':
    # Initialize the node
    br = CvBridge()
    augmented_reality_node = Augmenter(node_name='augmenter')
    # Keep it spinning to keep the node alive
    rospy.spin()    