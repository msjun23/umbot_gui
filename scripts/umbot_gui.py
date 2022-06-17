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
        self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        
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
                pose.pose.position.x = -9.45680046082
                pose.pose.position.y = 10.6780653
                pose.pose.orientation.z = 0.093994140625
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '302'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = -20.4987602234
                pose.pose.position.y = 12.1998758316
                pose.pose.orientation.z = -0.00143432617188
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '303'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = -15.8784141541
                pose.pose.position.y = 12.0001602173
                pose.pose.orientation.z = 0.109862558544
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '304'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = -13.9042387009
                pose.pose.position.y = 11.9014520645
                pose.pose.orientation.z = 0.11572265625
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '305'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = -8.97585487366
                pose.pose.position.y = 11.793056488
                pose.pose.orientation.z = 0.00253295898438
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '306'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = -7.32995033264
                pose.pose.position.y = 11.811003685
                pose.pose.orientation.z = 0.105865478516
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '307'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = 0.397379368544
                pose.pose.position.y = 11.9469299316
                pose.pose.orientation.z = 0.103881835938
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '308'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = 2.4981970787
                pose.pose.position.y = 11.8201189041
                pose.pose.orientation.z = 0.0979301854968
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '309'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = 10.9027824402
                pose.pose.position.y = 11.874256134
                pose.pose.orientation.z = 0.103911630809
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '310'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = 12.9858951569
                pose.pose.position.y = 11.8106479645
                pose.pose.orientation.z = 0.0740959569812
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '311'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = 17.5408782959
                pose.pose.position.y = 11.6658201218
                pose.pose.orientation.z = 0.014404296875
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '312'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = 24.1168251038
                pose.pose.position.y = 11.7474870682
                pose.pose.orientation.z = 0.175414308906
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '313'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = 27.7289600372
                pose.pose.position.y = 11.6660509109
                pose.pose.orientation.z = 0.332366943359
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '314'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = 29.0072956085
                pose.pose.position.y = 11.7294921875
                pose.pose.orientation.z = 0.366149157286
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '315'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = 34.1681747437
                pose.pose.position.y = 11.7369480133
                pose.pose.orientation.z = 0.425750732422
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '316'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = 27.338968277
                pose.pose.position.y = 10.2042541504
                pose.pose.orientation.z = 0.356201171875
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '317'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = 27.213312149
                pose.pose.position.y = -4.3870844841
                pose.pose.orientation.z = 0.395935058594
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '318'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = 18.9441375732
                pose.pose.position.y = -6.03807210922
                pose.pose.orientation.z = 0.411712646484
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '319'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = 16.8883724213
                pose.pose.position.y = -6.06414604187
                pose.pose.orientation.z = 0.395935058594
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '320'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = 2.90334129333
                pose.pose.position.y = -6.24777936935
                pose.pose.orientation.z = 0.390035400391
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '322'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = -2.65643072128
                pose.pose.position.y = -6.25700044632
                pose.pose.orientation.z = 0.389923095703
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '326'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = -9.54110240936
                pose.pose.position.y = -4.63662528992
                pose.pose.orientation.z = 0.395874023438
                pose.pose.orientation.w = 0.0

                self.goal_pub.publish(pose)
            elif (dest == '325'):
                self.mode = 'deli_ing'
                GPIO.output(action_sig, GPIO.HIGH)
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = -13.6711292267
                pose.pose.position.y = -4.80670976639
                pose.pose.orientation.z = 0.586608886719
                pose.pose.orientation.w = 0.0

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
            GPIO.output(action_sig, GPIO.HIGH)
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = -9.45680046082
            pose.pose.position.y = 10.6780653
            pose.pose.orientation.z = 0.093994140625
            pose.pose.orientation.w = 0.0
            self.goal_pub.publish(pose)
        else:
            rospy.loginfo('[Error] Equip Delivery module first, or delivery in on going')
            
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
    