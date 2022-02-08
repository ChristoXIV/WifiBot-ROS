#!/usr/bin/env python3
import sys
import time
import os
from uuid import uuid1
import random
# ROS imports
import rospy
import rospkg
from std_msgs.msg import Float64
# Move
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from sensor_msgs.msg import Image
# Fix to avoid bug that ros kinetic has
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
import cv2
import numpy as np

# Imports for TRaining
import torch
import torch.optim as optim
import torch.nn.functional as F
import torchvision
import torchvision.datasets as datasets
import torchvision.models as models
import torchvision.transforms as transforms



class IgnisBotCollisionAvoidance(object):

    def __init__(self, device_to_use = "cpu", csi_camera=True, plot_images=False, simulated_camera=False, camera_pan_angle = -0.605, camera_topic = "/jetdog/cam/image_raw", best_model_name = "best_model.pth"):

        self._device_to_use = device_to_use
        self._csi_camera = csi_camera
        self._plot_images = plot_images
        self._simulated_camera = simulated_camera
        #self.WIDTH = 224
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

        self.trained_models_path = os.path.join(self.collision_avoidance_pkg_path,'trained_models')
        self._best_model_name = best_model_name

        rospy.loginfo("Init IgnisBotCollisionAvoidance done...")

    def start_camera_servo(self):
        # Movement Topics publishers
        #self.camera_pan_publisher_topic_name = "/jetdog/camera_tilt_joint_position_controller/command"
        #self.camera_tilt_pub = rospy.Publisher(self.camera_pan_publisher_topic_name, Float64, queue_size=1)
        #self._check_pub_connection(self.camera_tilt_pub)
        #self.reset_camera_pan()
        print()

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
        except CvBridgeError as e:
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
        self.init_start_data_collection()
        if not self._simulated_camera:
            self.start_collision_avoidance_data_collection()
        else:
            rospy.spin()

    def start_training_collision_avoidance(self, num_epochs=30, num_workers=1, batch_size=16 ):


        rospy.logwarn("NUMBER OF WORKERS=="+str(num_workers))
        if self._device_to_use == "cpu":
            if num_workers > 1:
                rospy.logerr("NUMBER OF WORKERS=="+str(num_workers)+" Bigger than 1, and using cpu, you ight have issues if shared memory isnt enough")

        dataset_dir_exists = os.path.isdir(self.dataset_dir)
        if dataset_dir_exists:
            dataset = datasets.ImageFolder(
                self.dataset_dir,
                transforms.Compose([
                    transforms.ColorJitter(0.1, 0.1, 0.1, 0.1),
                    transforms.Resize((224, 224)),
                    transforms.ToTensor(),
                    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
                ])
            )

            train_dataset, test_dataset = torch.utils.data.random_split(dataset, [len(dataset) - 50, 50])

            train_loader = torch.utils.data.DataLoader(
                train_dataset,
                batch_size=batch_size,
                shuffle=True,
                num_workers=num_workers
            )

            test_loader = torch.utils.data.DataLoader(
                test_dataset,
                batch_size=batch_size,
                shuffle=True,
                num_workers=num_workers
            )

            # We load a pretrained model (with thousands of imges )to the transfer the learning to it
            model = models.alexnet(pretrained=True)
            # We add a blanc layer with only two outputs : free or blocked
            model.classifier[6] = torch.nn.Linear(model.classifier[6].in_features, 2)
            device = torch.device(self._device_to_use)
            model = model.to(device)
            self.start_training_model(model,train_loader,test_loader, device, test_dataset, num_epochs,self.trained_models_path, self._best_model_name)
        else:
            rospy.logerr("Folder Datasets doesnt exist, please generate it."+str(self.dataset_dir))

    def start_training_model(self, model, train_loader,test_loader, device, test_dataset, num_epochs, trained_models_path='', best_model_name="best_model.pth"):

        NUM_EPOCHS = num_epochs

        # We check if trained models folder dataset_dir_exists
        rospy.loginfo("Making directory for trained models")
        trained_models_dir_exists = os.path.isdir(trained_models_path)
        if not trained_models_dir_exists:
            os.makedirs(trained_models_path)

        BEST_MODEL_PATH = os.path.join(trained_models_path, best_model_name)
        best_accuracy = 0.0

        rospy.loginfo("Start Optimiser...")
        optimizer = optim.SGD(model.parameters(), lr=0.001, momentum=0.9)
        rospy.loginfo("Start Optimiser...DONE")

        for epoch in range(NUM_EPOCHS):
            epoch_start_seconds = rospy.get_time()
            rospy.loginfo("Start Train loader loop")
            length_train = len(train_loader)
            train_image_index = 1
            for images, labels in iter(train_loader):
                images = images.to(device)
                labels = labels.to(device)
                optimizer.zero_grad()
                outputs = model(images)
                loss = F.cross_entropy(outputs, labels)
                loss.backward()
                optimizer.step()
                rospy.loginfo("Train Images done="+str(train_image_index)+"/"+str(length_train))
                train_image_index += 1

            rospy.loginfo("Start Test loader loop")
            test_error_count = 0.0
            length_test = len(test_loader)
            test_image_index = 1
            for images, labels in iter(test_loader):
                images = images.to(device)
                labels = labels.to(device)
                outputs = model(images)
                test_error_count += float(torch.sum(torch.abs(labels - outputs.argmax(1))))

                rospy.loginfo("Test Images done="+str(test_image_index)+"/"+str(length_test))
                test_image_index += 1

            test_accuracy = 1.0 - float(test_error_count) / float(len(test_dataset))
            rospy.loginfo('EPOCH=%d: TEST ACCURACY=%f' % (epoch, test_accuracy))
            if test_accuracy > best_accuracy:
                rospy.loginfo(">>>>>Saving Better Model...")
                torch.save(model.state_dict(), BEST_MODEL_PATH)
                rospy.loginfo(">>>>>Saving Better Model...DONE")
                best_accuracy = test_accuracy

            epoch_end_seconds = rospy.get_time()
            delta_epoch = epoch_end_seconds - epoch_start_seconds

            time_left_training = delta_epoch * (NUM_EPOCHS - epoch )
            rospy.loginfo("TIME ESTIMATED LEFT OF TRAINING (seconds) ==>"+str(time_left_training))

    def start_collision_prediction(self, max_angular_speed, move_speed, simulated_camera=False, camera_pan_angle = -0.605, camera_topic = "/jetdog/cam/image_raw"):

        self.start_camera_servo()

        self._max_angular_speed = max_angular_speed
        self._move_speed = move_speed

        self.cmd_vel_pub = rospy.Publisher("/robot/cmd_vel", Twist, queue_size=1)

        model = torchvision.models.alexnet(pretrained=False)
        model.classifier[6] = torch.nn.Linear(model.classifier[6].in_features, 2)
        # We load the weights of the bets model of the training
        # We check if trained models folder dataset_dir_exists
        best_model_file_path = os.path.join(self.trained_models_path, self._best_model_name)
        rospy.loginfo("Checking there is the better model file")

        best_model_file_exists = os.path.isfile(best_model_file_path)
        if best_model_file_exists:
            model.load_state_dict(torch.load(best_model_file_path))
            self.device = torch.device(self._device_to_use)
            self.model = model.to(self.device)

            # We have to preprocess images to adapt to the model Training
            mean = 255.0 * np.array([0.485, 0.456, 0.406])
            stdev = 255.0 * np.array([0.229, 0.224, 0.225])

            self.normalize = torchvision.transforms.Normalize(mean, stdev)

            if simulated_camera:
                rospy.logwarn("Simulated Camera")
                from cv_bridge import CvBridge, CvBridgeError

                self.cv_image = None

                self.bridge_object = CvBridge()
                self.image_sub = rospy.Subscriber(self._camera_topic,Image,self.camera_callback)

                self.start_sim_collision_avoidance()

            else:
                if self._csi_camera:
                    from jetcam.csi_camera import CSICamera
                    self.camera = CSICamera(width=self.WIDTH, height=self.HEIGHT, capture_fps=30)
                else:
                    from jetcam.usb_camera import USBCamera
                    self.camera = USBCamera(capture_device=0)

                self.start_collision_avoidance()


        else:
            rospy.logerr("Weight Best model file doesnt exist="+str(best_model_file_path))

    def start_sim_collision_avoidance(self):
        rospy.loginfo("Starting SIM Loop...")

        # Start a random turn
        self.move_robot(direction="turn")
        rospy.logwarn("Random turn...")
        time.sleep(random.random()*10.0)
        rospy.logwarn("Random turn...END")


        while not rospy.is_shutdown():
            rospy.logdebug("Sleeping")
            # Time until next prediction
            time.sleep(0.05)
            rospy.logdebug("Sleeping...DONE")

            if self.cv_image is not None:
                rospy.logdebug("Processing Image...")
                before_seconds = rospy.get_time()
                # We format image for neuralnetwork
                x = self.pre_process_image(self.cv_image)
                y = self.model(x)
                # we apply the `softmax` function to normalize the output vector so it sums to 1 (which makes it a probability distribution)
                y = F.softmax(y, dim=1)

                prob_blocked = float(y.flatten()[0])

                if prob_blocked < 0.5:
                    rospy.loginfo("Move Fowards="+str(prob_blocked)+"< 0.5")
                    self.move_robot(direction="forwards")
                else:
                    rospy.loginfo("Move Turn="+str(prob_blocked)+">= 0.5")
                    self.move_robot(direction="turn")

                after_seconds = rospy.get_time()
                delta = after_seconds - before_seconds
                rospy.logdebug("CPU Processing Done, Seconds Time="+str(delta))



        rospy.loginfo("Terminating Loop...")

    def start_collision_avoidance(self):
        rospy.loginfo("Starting Loop...")
        self.camera.running = True
        self.camera.observe(self.execute, names='value')
        rospy.spin()
        rospy.loginfo("Terminating Loop...")

    def execute(self, change):

        image = change['new']
        before_seconds = rospy.get_time()
        x = self.pre_process_image(image)
        y = self.model(x)


        # we apply the `softmax` function to normalize the output vector so it sums to 1 (which makes it a probability distribution)
        y = F.softmax(y, dim=1)

        prob_blocked = float(y.flatten()[0])

        if prob_blocked < 0.5:
            rospy.loginfo("Move Fowards="+str(prob_blocked)+"< 0.5")
            self.move_robot(direction="forwards")
        else:
            rospy.loginfo("Move Turn="+str(prob_blocked)+">= 0.5")
            self.move_robot(direction="turn")

        after_seconds = rospy.get_time()
        delta = after_seconds - before_seconds
        #rospy.loginfo("GPU Processing Done, Seconds Time="+str(delta))

    def pre_process_image(self, camera_value):

        x = camera_value
        x = cv2.cvtColor(x, cv2.COLOR_BGR2RGB)
        x = x.transpose((2, 0, 1))
        x = torch.from_numpy(x).float()
        x = self.normalize(x)
        x = x.to(self.device)
        x = x[None, ...]

        # Draw New Image
        if self._plot_images:
            cv2.imshow("CollisionAvoid", camera_value)
            cv2.waitKey(1)

        return x

    def move_robot(self, direction):

        if direction == "forwards":
            self.move_cmd_vel(linear=self._move_speed,angular=0.0)

        else:
            self.move_cmd_vel(linear=0.0,angular=self._max_angular_speed)

    def move_cmd_vel(self, linear,angular):
        cmd_vel_obj = Twist()
        cmd_vel_obj.linear.x = linear
        cmd_vel_obj.angular.z = angular
        self.cmd_vel_pub.publish(cmd_vel_obj)


if __name__ == "__main__":
    rospy.init_node("collision_avoidance_training_test_node", log_level=rospy.INFO)
    if len(sys.argv) < 9:
        print("usage: collision_avoidance_training.py simulated_camera camera_pan_angle mode device_to_use csi_camera plot_images max_angular_speed move_speed")
    else:
        simulated_camera = bool(sys.argv[1] == "true")
        camera_pan_angle = float(sys.argv[2])
        mode = sys.argv[3]
        device_to_use = sys.argv[4]
        csi_camera = (sys.argv[5] == "true")
        plot_images = (sys.argv[6] == "true")
        max_angular_speed = float(sys.argv[7])
        move_speed = float(sys.argv[8])

        ca_object = IgnisBotCollisionAvoidance( device_to_use=device_to_use,
                                                csi_camera=csi_camera,
                                                plot_images=plot_images,
                                                simulated_camera=simulated_camera,
                                                camera_pan_angle= camera_pan_angle)
        if mode == "get_data":
            rospy.loginfo("Started GET DATA")
            ca_object.start_dataget_collision_avoidance()
        elif mode == "train":
            if len(sys.argv) < 12:
                print("usage: collision_avoidance_training.py simulated_camera camera_pan_angle mode device_to_use csi_camera plot_images max_angular_speed move_speed num_epochs workers batch_size")
            else:
                rospy.loginfo("Started TRAINING")
                num_epochs = int(float(sys.argv[9]))
                workers = int(float(sys.argv[10]))
                batch_size = int(float(sys.argv[11]))
                ca_object.start_training_collision_avoidance(num_epochs=30,
                                                             num_workers=workers,
                                                             batch_size=batch_size)
        elif mode == "predict":
            rospy.loginfo("Started Predicting")
            ca_object.start_collision_prediction(max_angular_speed=max_angular_speed,
                                                 move_speed=move_speed,
                                                 simulated_camera=simulated_camera,
                                                 camera_pan_angle = camera_pan_angle)
        else:
            pass
