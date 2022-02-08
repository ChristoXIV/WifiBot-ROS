#!/usr/bin/env python3
import sys
import time
import os
# ROS imports
import rospy
from tkinter import *
from std_msgs.msg import Int32
from PIL import ImageTk, Image
import rospkg
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse


class CollisionAvoidanceDataCollectionGUI(object):

    def __init__(self, simulated_camera=False, camera_topic = "/jetdog/cam/image_raw"):

        self.service_name = '/ignisbot/collision_avoidance_datacollection'
        self.request = SetBoolRequest()

        rospy.logwarn("Waiting for service ="+str(self.service_name))
        rospy.wait_for_service(self.service_name)
        rospy.logwarn("Service FOUND ="+str(self.service_name))

        self.save_data_image_srv_client = rospy.ServiceProxy(self.service_name, SetBool)

        self.window = Tk()
        self.window.title("Collision Avoidance Ignisbot app")
        self.window.geometry('400x400')

        rospy.loginfo("Getting Path to package...")
        collision_avoidance_pkg_path = rospkg.RosPack().get_path('projet_mecatronique')
        self.blocked_dir = os.path.join(collision_avoidance_pkg_path,'dataset/blocked')
        self.free_dir = os.path.join(collision_avoidance_pkg_path,'dataset/free')
        init_image_path = os.path.join(collision_avoidance_pkg_path,'scripts/ria.png')
        init_image_path_2 = os.path.join(collision_avoidance_pkg_path,'scripts/tc.jpg')

        self._block_img = ImageTk.PhotoImage(Image.open(init_image_path_2))
        self._free_img = ImageTk.PhotoImage(Image.open(init_image_path))

        self.img_panel = Label(self.window, image = self._free_img)
        self.img_panel.grid(column=2, row=0)

        self._free = 0
        self._block = 0

        btn_h = 5
        btn_w = 5

        self.lbl_free = Label(self.window, text="FREE = "+str(self._free))
        self.lbl_free.grid(column=0, row=0)
        self.lbl_block = Label(self.window, text="BLOCK = "+str(self._block))
        self.lbl_block.grid(column=0, row=1)

        # We update the lables based on the files
        self.update_file_numbers()

        self.btn_free = Button(self.window, text="FREE", command=self.free_clicked, bg="green", fg="white", height = btn_h, width = btn_w)
        self.btn_free.grid(column=1, row=0)

        self.btn_block = Button(self.window, text="BLOCK", command=self.block_clicked, bg="red", fg="white", height = btn_h, width = btn_w)
        self.btn_block.grid(column=1, row=1)

        self.window.mainloop()

    def free_clicked(self):
        # True if free, false if blocked
        self.request.data = True
        response = self.save_data_image_srv_client(self.request)
        saved_image_path = response.message
        file_exists = os.path.isfile(saved_image_path)
        rospy.loginfo("Image_path="+str(saved_image_path)+",=>"+str(file_exists))

        # It has to be an image class that isnt detroyed outside this method
        self._free_img = ImageTk.PhotoImage(Image.open(saved_image_path))
        self.img_panel.configure(image = self._free_img)

        self.update_file_numbers()


    def block_clicked(self):
        self.request.data = False
        response = self.save_data_image_srv_client(self.request)
        saved_image_path = response.message
        file_exists = os.path.isfile(saved_image_path)
        rospy.loginfo("Image_path="+str(saved_image_path)+",=>"+str(file_exists))

        # It has to be an image class that isnt detroyed outside this method
        self._block_img = ImageTk.PhotoImage(Image.open(saved_image_path))
        self.img_panel.configure(image = self._block_img)

        self.update_file_numbers()

    def update_file_numbers(self):

        try:
            self._free = len(os.listdir(self.free_dir))
            self._block = len(os.listdir(self.blocked_dir))

        except FileNotFoundError:
            self._free = 0
            self._block = 0

        self.lbl_free.configure(text="FREE = "+str(self._free))
        self.lbl_block.configure(text="BLOCK = "+str(self._block))



if __name__ == "__main__":
    rospy.init_node("simplegui_node", log_level=rospy.INFO)
    gui_object = CollisionAvoidanceDataCollectionGUI()