#!/usr/bin/env python3.6

from distutils.command.clean import clean
import rospy
import RPi.GPIO as GPIO
import sys

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist

from kivy.lang import Builder
from kivymd.app import MDApp
from kivy.core.window import Window

# Window.size = (1280, 960)

nuri_background = """
Image: 
    source: "/home/msjun-xavier/catkin_ws/src/umbot_gui/img/NURI.png"
"""

# Pin number
deli_module = 15
disinf_module = 16
clean_module = 18
action_sig = 999

class UmbotGUI(MDApp):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(deli_module, GPIO.IN)
        GPIO.setup(disinf_module, GPIO.IN)
        GPIO.setup(clean_module, GPIO.IN)
        GPIO.setup(action_sig, GPIO.output)
        
        rospy.init_node('umbot_gui',anonymous=True)
        
        self.mode = 'non'      # non, deli_wait, deli_ing, disinfection, cleaning
        self.pw = ''

        self.mode_pub = rospy.Publisher('/umbot_mode', String, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        
        kivy_file = rospy.get_param('~kivy_file')      # Get kivy file
        print(kivy_file)
        self.kivy_main = Builder.load_file(kivy_file)

    def build(self):
        return self.kivy_main
    
    ################################################
    # Delivery mode #
    def btnOpen_pressed(self, *args):
        if (self.mode == 'deli_ing'):
            pw = self.kivy_main.ids.textPassword.text
            if (pw == self.pw):
                rospy.loginfo('Open deli bucket')
                
                self.mode = 'deli_wait'
                
                rospy.loginfo('Go back to station')
                # Go back to station
                # pose = PoseStamped()
                # pose.header.frame_id = 'map'
                # pose.pose.position.x = -0.379455089569
                # pose.pose.position.y = 8.80924701691
                # pose.pose.orientation.z = 0.998416539354
                # pose.pose.orientation.w = -0.0562531238558

                # self.goal_pub.publish(pose)
            else:
                rospy.loginfo('[Error] Password is wrong! Try again')
        else:
            rospy.loginfo('[Error] Equip Delivery module first, or delivery is not on going')
        
    def btnSet_pressed(self, *args):
        # if (self.mode == 'deli_wait'):        # deli_wait
        pin_deli = GPIO.input(deli_module)
        if (pin_deli == 1):
            dest = self.kivy_main.ids.textRoom.text
            self.pw = self.kivy_main.ids.textPassword.text
            
            rospy.loginfo('Go to destination: ' + dest)
            
            # Publish the goal point as PoseStamped
            if (dest == '301'):
                self.mode = 'deli_ing'
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = -0.379455089569
                pose.pose.position.y = 8.80924701691
                pose.pose.orientation.z = 0.998416539354
                pose.pose.orientation.w = -0.0562531238558

                self.goal_pub.publish(pose)
            elif (dest == '302'):
                self.mode = 'deli_ing'
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = -0.379455089569
                pose.pose.position.y = 8.80924701691
                pose.pose.orientation.z = 0.998416539354
                pose.pose.orientation.w = -0.0562531238558

                self.goal_pub.publish(pose)
            else:
                rospy.loginfo('[Error] ' + dest + ' is not existed!')
        else:
            rospy.loginfo('[Error] Equip Delivery module first, or delivery in on going')
        
    def btnRoom301_pressed(self, *args):
        # if (self.mode == 'deli_wait'):            # deli_wait
        pin_deli = GPIO.input(deli_module)
        if (pin_deli == 1):
            rospy.loginfo('Go to Dr.Oh')
            self.mode = 'deli_ing'
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = -0.379455089569
            pose.pose.position.y = 8.80924701691
            pose.pose.orientation.z = 0.998416539354
            pose.pose.orientation.w = -0.0562531238558

            self.goal_pub.publish(pose)
        else:
            rospy.loginfo('[Error] Equip Delivery module first, or delivery in on going')
            
    ################################################
    # Disinfection mode #
    def btnDisinfection_pressed(self, *args):
        if (self.mode == 'non'):                # disinfection
        # pin_disinf = GPIO.input(disinf_module)
        # if (pin_disinf == 1):
            rospy.loginfo('Disinfection mode is running')
            
            self.mode_pub.publish('disinfection')
        else:
            rospy.loginfo('[Error] Equip Disinfection module first!!!')
            
    ################################################
    # Cleaning mode #
    def btnCleaning_pressed(self, *args):
        if (self.mode == 'non'):                # cleaning
        # pin_clean = GPIO.input(clean_module)
        # if (pin_clean == 1):
            rospy.loginfo('Cleaning mode is running')
            
            self.mode_pub.publish('cleaning')
        else:
            rospy.loginfo('[Error] Equip Cleaning module first!!!')
            
    ################################################
    # Stop #
    def btnStop_pressed(self, *args):
        rospy.loginfo('Stopping')
        
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        cmd_vel = Twist()
        
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = 0.0
        
        pub.publish(cmd_vel)
        self.mode_pub.publish('stop')

if __name__=='__main__':
    GUI = UmbotGUI()
    GUI.run()
    