#!/usr/bin/env python3.6

import rospy

from kivy.lang import Builder
from kivymd.app import MDApp

nuri_background = """
Image: 
    source: "/home/msjun-xavier/catkin_ws/src/umbot_gui/img/NURI.png"
"""

class UmbotGUI(MDApp):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        kivy_file = '/home/msjun-xavier/catkin_ws/src/umbot_gui/kivy/nuri.kv'
        self.img = Builder.load_file(kivy_file)
        # self.img = Builder.load_string(nuri_background)

    def build(self):
        return self.img
    
    def btnRoom322_pressed(self, *args):
        print("Go to Room 322")
        
    def btnRoom320_pressed(self, *args):
        print("Go to RAIL")
        
    def btnRoom318_pressed(self, *args):
        print("Go to CDSL")
        
    def btnRoom301_pressed(self, *args):
        print("Go to Dr.Oh")
        
    def btnRoom309_pressed(self, *args):
        print("Go to 309")
        
    def btnRoom314_pressed(self, *args):
        print("Go to Dr.Jung")

if __name__=='__main__':
    GUI = UmbotGUI()
    GUI.run()
    