#!/usr/bin/env python3.6

import rospy

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from kivy.lang import Builder
from kivymd.app import MDApp

nuri_background = """
Image: 
    source: "/home/msjun-xavier/catkin_ws/src/umbot_gui/img/NURI.png"
"""

class UmbotGUI(MDApp):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        rospy.init_node('delivery_gui',anonymous=True)

        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        
        kivy_file = rospy.get_param('~kivy_file')      # Get kivy file
        print(kivy_file)
        self.kivy_main = Builder.load_file(kivy_file)
        # self.img = Builder.load_string(nuri_background)

    def build(self):
        return self.kivy_main
    
    def btnRoom322_pressed(self, *args):
        print("Go to Room 322")
        
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = 3.12410187721
        pose.pose.position.y = 3.88758349419
        pose.pose.orientation.z = -0.540481435978
        pose.pose.orientation.w = 0.841355939756
        
        self.goal_pub.publish(pose)
        
    def btnRoom320_pressed(self, *args):
        print("Go to RAIL")
        
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = 8.40724086761
        pose.pose.position.y = -0.867243170738
        pose.pose.orientation.z = -0.718212506596
        pose.pose.orientation.w = 0.695823824951
        
        self.goal_pub.publish(pose)
                
    def btnRoom318_pressed(self, *args):
        print("Go to CDSL")
        
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = 2.09527802467
        pose.pose.position.y = -7.42946052551
        pose.pose.orientation.z = 0.994984225062
        pose.pose.orientation.w = 0.100031954288
        
        self.goal_pub.publish(pose)
        
    def btnRoom301_pressed(self, *args):
        print("Go to Dr.Oh")
        
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = -4.77280473709
        pose.pose.position.y = 0.99575984478
        pose.pose.orientation.z = 0.777963984101
        pose.pose.orientation.w = 0.628308872643
        
        self.goal_pub.publish(pose)
        
    def btnRoom309_pressed(self, *args):
        print("Go to 309")
        
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = -5.63479042053
        pose.pose.position.y = -4.37079906464
        pose.pose.orientation.z = 0.933511747704
        pose.pose.orientation.w = -0.358546812701
        
        self.goal_pub.publish(pose)
        
    def btnRoom314_pressed(self, *args):
        print("Go to Dr.Jung")
        
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = -0.379455089569
        pose.pose.position.y = 8.80924701691
        pose.pose.orientation.z = 0.998416539354
        pose.pose.orientation.w = -0.0562531238558

        self.goal_pub.publish(pose)

if __name__=='__main__':
    GUI = UmbotGUI()
    GUI.run()
    