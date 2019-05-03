#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from keras.models import load_model
import cv2 as cv2

global_speed=0.3
class AIController:
    def __init__(self):
        rospy.init_node("ai_controller")

        self.bridge = CvBridge()

        # subscribe to images
        #rospy.Subscriber("/image_publisher/image_raw", Image, self.on_image)
        rospy.Subscriber("/conde_camera_signalling_panel/image_raw", Image, self.on_image)
        # this subsriber is the safety switch
        #rospy.Subscriber("/vesc/ai", Int8, self.on_ai)

        # model is located one dir up
        self.model = load_model('./driving_net_model_sim.h5')
        rospy.logwarn("Model successfully loaded")
        # hijack the cmd_vel topic
        #self.teleop_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
        self.teleop_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.image = None
        self.angular_z = 0
        self.run_nn()
        # rospy.spin()


    def run_nn(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.image is not None:
                rospy.loginfo("Image has been detected")
                try:
                    # start = rospy.get_time()
                    img = self.image[200:, :]
                    img = cv2.resize(img, (168, 44), interpolation=cv2.INTER_AREA)
                    prediction = self.model.predict(img[None, :, :, :], batch_size=1)[0][0]
                    self.angular_z = prediction
                    # rospy.logwarn(prediction)
                    # rospy.logwarn("time elapsed doing work: %s", rospy.get_time() - start)
                    self.publish_teleop()
                except CvBridgeError as e:
                    rospy.logerr(e)
                except Exception as e:
                    rospy.logerr(e)

            rate.sleep()
            
    def on_image(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def on_ai(self, data):
        self.publish_teleop()

    def publish_teleop(self):
        steering = self.angular_z
        cmd_vel = self.generate_twist(steering,global_speed)
        self.teleop_pub.publish(cmd_vel)


#Need to fix speed so that it comes from a topic
    def generate_twist(self, steering, speed):
        #stamp = AckermannDriveStamped()
        cmd_vel = Twist()

        cmd_vel.linear.x = speed
        cmd_vel.linear.y = 0
        cmd_vel.linear.z = 0
        cmd_vel.angular.x = 0
        cmd_vel.angular.y = 0
        cmd_vel.angular.z = steering
        rospy.logwarn("steering: %s", steering)
        return cmd_vel




if __name__ == "__main__":
    try:
        AIController()
    except rospy.ROSInterruptException:
        rospy.logerr("Could not start AI Controller")
