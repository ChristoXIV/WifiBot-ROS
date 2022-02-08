#!/usr/bin/env python3
import sys
import time
import os
from uuid import uuid1
# ROS imports
import rospy
import rospkg
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
# Fix to avoid bug that ros kinetic has
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
import cv2



class IgnisBotCollisionAvoidance(object):

    def __init__(self, device_to_use = "cuda", csi_camera=True, plot_images=False, simulated_camera=False, camera_pan_angle = -0.605, camera_topic = "/jetdog/cam/image_raw"):

        self._device_to_use = device_to_use
        self._csi_camera = csi_camera
        self._plot_images = plot_images
        self._simulated_camera = simulated_camera
        #self.WIDTH = 224    # Format accepter par le modèle d'entrainement.
        #self.HEIGHT = 224
        self._camera_topic = camera_topic
        self._camera_pan_angle = camera_pan_angle

        self.free_count = 0
        self.block_count = 0

        # Create Paths for storing images collected
        rospy.loginfo("Getting Path to package...")
        self.collision_avoidance_pkg_path = rospkg.RosPack().get_path('projet_mecatronique') 
        self.dataset_dir = os.path.join(self.collision_avoidance_pkg_path,'dataset')
        self.blocked_dir = os.path.join(self.collision_avoidance_pkg_path,'dataset/blocked')
        self.free_dir = os.path.join(self.collision_avoidance_pkg_path,'dataset/free')

        rospy.loginfo("Init IgnisBotCollisionAvoidance done...")
        
    
    def _check_pub_connection(self, publisher_object):

        rate = rospy.Rate(10)  # 10hz
        while publisher_object.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to publisher_object yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("publisher_object Publisher Connected")

        rospy.logdebug("All Publishers READY")


    def start_camera_servo(self):
        # Movement Topics publishers
        #self.camera_pan_publisher_topic_name = "/jetdog/camera_tilt_joint_position_controller/command"
        #self.camera_tilt_pub = rospy.Publisher(self.camera_pan_publisher_topic_name, Float64, queue_size=1)
        #self._check_pub_connection(self.camera_tilt_pub)
        #self.reset_camera_pan()
        print()

    def reset_camera_pan(self):
        """
        Reseting Camera pan to the angle that will be used to navigate
        """
        self.move_camera_pan(self._camera_pan_angle)

    def move_camera_pan(self, angle):
        pan_angle_obj = Float64()
        pan_angle_obj.data= angle
        self.camera_tilt_pub.publish(pan_angle_obj)


    def handle_ca_signal(self, req):

        is_free = req.data

        if is_free:
            info = self.save_free()
        else:
            info = self.save_blocked()

        response = SetBoolResponse()
        response.success = True
        # We return the path to the image stored
        response.message = info

        return response

    def camera_callback(self,data):
        
        try:
            # We select bgr8 because its the OpneCV encoding by default
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except cv2.CvBridgeError as e:
            print(e)

    def init_start_data_collection(self):
       
        self.start_camera_servo()
        
        s = rospy.Service('/ignisbot/collision_avoidance_datacollection', SetBool, self.handle_ca_signal)
        
        if self._simulated_camera:
            
            rospy.logwarn("Simulated Camera")
            from cv_bridge import CvBridge, CvBridgeError

            self.cv_image = None

            self.bridge_object = CvBridge()
            self.image_sub = rospy.Subscriber(self._camera_topic,Image,self.camera_callback)

        else:
            # Its the physical robot
            # TODO: This has to be tested and debuged
            if self._csi_camera:
                from jetcam.csi_camera import CSICamera
                self.camera = CSICamera(width=self.WIDTH, height=self.HEIGHT, capture_fps=30)
            else:
                
                from jetcam.usb_camera import USBCamera
                self.camera = USBCamera(capture_device=0)

        # we have this "try/except" statement because these next functions can throw an error if the directories exist already
        try:
            os.makedirs(self.free_dir)
            os.makedirs(self.blocked_dir)
        except FileExistsError:
            rospy.logerr('Directories not created because they already exist')

    def start_collision_avoidance_data_collection(self):
        rospy.loginfo("Starting Real IgnisBot Data collection Loop...")
        self.camera.running = True
        self.camera.observe(self.execute_datacolection, names='value')
        rospy.spin()
        rospy.loginfo("Terminating Loop...")

    def execute_datacolection(self, change):
        # We update cv_image with the latest image value
        self.cv_image = change['new']
        

    def save_snapshot(self, directory):
        image_path = os.path.join(directory, str(uuid1()) + '.jpg')
        with open(image_path, 'wb') as f:
            cv2.imwrite(image_path, self.cv_image)            

        return image_path

    def save_free(self):

        info = self.save_snapshot(self.free_dir)
        self.free_count = len(os.listdir(self.free_dir))
        return info

    def save_blocked(self):

        info = self.save_snapshot(self.blocked_dir)
        self.block_count = len(os.listdir(self.blocked_dir))
        return info
    
    def start_dataget_collision_avoidance(self):
         # 
        self.init_start_data_collection()
        if not self._simulated_camera:
            
            self.start_collision_avoidance_data_collection()
        else:
            print("test: ",self._simulated_camera)
            rospy.spin()



if __name__ == "__main__":
    
    rospy.init_node("collision_avoidance_training_test_node", log_level=rospy.INFO)
    if len(sys.argv) < 9:
        print("usage: collision_avoidance_training.py simulated_camera camera_pan_angle mode device_to_use csi_camera plot_images")
    else:
        
        #simulated_camera = bool(sys.argv[1]) 
        simulated_camera = bool(sys.argv[1]=="true")        # Indique si l'image vient du ros topic ou "jetbot camera class" 
        camera_pan_angle = float(sys.argv[2])                  # l'angle de la caméra
        mode = sys.argv[3]                    
        device_to_use = sys.argv[4]                            # Selon matériel CPU ou GPU
        csi_camera = (sys.argv[5]=="true")                   # Vérifie la connection de la caméra
        plot_images = (sys.argv[6]=="true")                  # Affiche la caméra sur le bureau 
        ca_object = IgnisBotCollisionAvoidance( device_to_use=device_to_use,
                                                csi_camera=csi_camera,
                                                plot_images=plot_images,
                                                simulated_camera=simulated_camera,
                                                camera_pan_angle= camera_pan_angle)
                                          
        if mode == "get_data":
            rospy.loginfo("Started GET DATA")
            
            ca_object.start_dataget_collision_avoidance()
        else:
            pass
