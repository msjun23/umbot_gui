#!/usr/bin/env python3.6

from distutils.command.clean import clean
import rospy
import RPi.GPIO as GPIO
import sys

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from actionlib_msgs.msg import GoalID

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
action_sig = 22

class UmbotGUI(MDApp):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(deli_module, GPIO.IN)
        GPIO.setup(disinf_module, GPIO.IN)
        GPIO.setup(clean_module, GPIO.IN)
        GPIO.setup(action_sig, GPIO.OUT)
        GPIO.output(action_sig, GPIO.LOW)
        
        rospy.init_node('umbot_gui',anonymous=True)
        
        self.mode = 'non'      # non, deli_wait, deli_ing, disinfection, cleaning
        self.pw = ''
        
        self.disinf_flag = 0

        self.mode_pub = rospy.Publisher('/umbot_mode', String, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        rospy.Subscriber('/spray_cmd', String, self.SpraySubscriber)
        
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
                
                self.mode = 'deli_wait'     # deli_wait
                GPIO.output(action_sig, GPIO.LOW)
                
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
        # if (self.mode == 'non'):        # deli_wait
        pin_deli = GPIO.input(deli_module)
        if (pin_deli == 1 and self.mode != 'deli_ing'):
            # Set destination
            dest = self.kivy_main.ids.textRoom.text
            # Set password
            self.pw = self.kivy_main.ids.textPassword.text
            
            rospy.loginfo('Go to destination: ' + dest)
            
            # Publish the goal point as PoseStamped
            if (dest == '301'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = -9.36647319794
                pose.pose.position.y = 10.9203100204
                pose.pose.orientation.z = -0.00143432617188
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '302'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = -20.6209869385
                pose.pose.position.y = 12.8988008499
                pose.pose.orientation.z = -0.00534057617188
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '303'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = -15.6618528366
                pose.pose.position.y = 12.8587036133
                pose.pose.orientation.z = -0.00534057617188
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '304'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = -13.8950843811
                pose.pose.position.y = 12.8689470291
                pose.pose.orientation.z = -0.00143432617188
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '305'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = -9.72575569153
                pose.pose.position.y = 12.6843156815
                pose.pose.orientation.z = -0.00143432617188
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '306'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = -7.21711778641
                pose.pose.position.y = 12.571480751
                pose.pose.orientation.z = -0.00143432617188
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '307'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = 0.274027734995
                pose.pose.position.y = 12.3133602142
                pose.pose.orientation.z = -0.00143432617188
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '308'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = 3.06015181541
                pose.pose.position.y = 12.2944278717
                pose.pose.orientation.z = -0.00143432617188
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '309'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = 10.7025022507
                pose.pose.position.y = 12.0021352768
                pose.pose.orientation.z = -0.00143432617188
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '310'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = 12.8394927979
                pose.pose.position.y = 11.8754043579
                pose.pose.orientation.z = -0.00143432617188
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '311'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = 17.2638759613
                pose.pose.position.y = 11.7383880615
                pose.pose.orientation.z = -0.00143432617188
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '312'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = 23.7804088593
                pose.pose.position.y = 11.4395561218
                pose.pose.orientation.z = -0.00143432617188
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '313'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = 27.7941646576
                pose.pose.position.y = 11.5442409515
                pose.pose.orientation.z = -0.00534057617188
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '314'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = 28.9799613953
                pose.pose.position.y = 11.516254425
                pose.pose.orientation.z = -0.00143432617188
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '315'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = 34.1715583801
                pose.pose.position.y = 11.4422483444
                pose.pose.orientation.z = -0.00143432617188
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '316'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = 28.2606868744
                pose.pose.position.y = 9.75775909424
                pose.pose.orientation.z = -0.00143432617188
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '317'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = 26.0412254333
                pose.pose.position.y = -4.10744857788
                pose.pose.orientation.z = -0.00143432617188
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '318'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = 18.9336547852
                pose.pose.position.y = -5.74415254593
                pose.pose.orientation.z = -0.00143432617188
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '319'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = 17.1751689911
                pose.pose.position.y = -5.71689224243
                pose.pose.orientation.z = -0.00143432617188
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '320'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = 2.93234229088
                pose.pose.position.y = -5.84798765182
                pose.pose.orientation.z = -0.00143432617188
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '322'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = -2.88115143776
                pose.pose.position.y = -5.92585039139
                pose.pose.orientation.z = -0.0052497391589
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '326'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = -7.61158084869
                pose.pose.position.y = -4.38990545273
                pose.pose.orientation.z = 0.00247192382812
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '325'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = -14.0445680618
                pose.pose.position.y = -4.32711601257
                pose.pose.orientation.z = 0.00247192382812
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            else:
                rospy.loginfo('[Error] ' + dest + ' is not existed!')
        else:
            rospy.loginfo('[Error] Equip Delivery module first, or delivery in on going')
        
    # def btnRoom301_pressed(self, *args):
    #     # if (self.mode == 'deli_wait'):            # deli_wait
    #     pin_deli = GPIO.input(deli_module)
    #     if (pin_deli == 1):
    #         rospy.loginfo('Go to Dr.Oh')
    #         self.mode = 'deli_ing'
    #         GPIO.output(action_sig, GPIO.HIGH)
            
    #         pose = PoseStamped()
    #         pose.header.frame_id = 'map'
    #         pose.pose.position.x = -9.45680046082
    #         pose.pose.position.y = 10.6780653
    #         pose.pose.orientation.z = 0.093994140625
    #         pose.pose.orientation.w = 0.0
    #         self.goal_pub.publish(pose)
    #     else:
    #         rospy.loginfo('[Error] Equip Delivery module first, or delivery in on going')
            
    ################################################
    # Disinfection mode #
    def btnDisinfection_pressed(self, *args):
        # if (self.mode == 'non'):                # disinfection
        pin_disinf = GPIO.input(disinf_module)
        if (pin_disinf == 1):
            rospy.loginfo('Disinfection mode is running')
            
            # if (self.disinf_flag == 0):
            #     self.disinf_flag = 1
            #     GPIO.output(action_sig, GPIO.HIGH)
            # else:
            #     self.disinf_flag = 0
            #     GPIO.output(action_sig, GPIO.LOW)
            
            self.mode_pub.publish('disinfection')
        else:
            rospy.loginfo('[Error] Equip Disinfection module first!!!')
            
    def SpraySubscriber(self, data):
        if (data.data == 'spray'):
            if (self.disinf_flag == 0):
                self.disinf_flag = 1
                GPIO.output(action_sig, GPIO.HIGH)
            else:
                self.disinf_flag = 0
                GPIO.output(action_sig, GPIO.LOW)
            
    ################################################
    # Cleaning mode #
    def btnCleaning_pressed(self, *args):
        # if (self.mode == 'non'):                # cleaning
        pin_clean = GPIO.input(clean_module)
        if (pin_clean == 1):
            rospy.loginfo('Cleaning mode is running')
            GPIO.output(action_sig, GPIO.HIGH)
            
            self.mode_pub.publish('cleaning')
        else:
            rospy.loginfo('[Error] Equip Cleaning module first!!!')
            
    ################################################
    # Stop #
    def btnStop_pressed(self, *args):
        rospy.loginfo('Stopping')
        pin_clean = GPIO.input(clean_module)
        if (pin_clean == 1):
            GPIO.output(action_sig, GPIO.LOW)
        
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        cmd_vel = Twist()
        cancel_msg = GoalID()
        
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = 0.0
        
        pub.publish(cmd_vel)
        self.mode_pub.publish('stop')
        self.cancel_pub.publish(cancel_msg)

if __name__=='__main__':
    GUI = UmbotGUI()
    GUI.run()
    