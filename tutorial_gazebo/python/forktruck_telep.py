#!/usr/bin/env python

import rospy
import wx

from math import radians

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


width = 325

class servoSlider():
    def __init__(self, parent, min_angle, max_angle, name, i):
        self.name = name
        self.position = wx.Slider(parent, -1, 0, int(min_angle*100), int(max_angle*100), wx.DefaultPosition, (150, -1), wx.SL_HORIZONTAL)
        self.enabled = wx.CheckBox(parent, i, name+":")
        self.enabled.SetValue(False)
        self.position.Disable()

    def setPosition(self, angle):
        self.position.SetValue(int(angle*100))

    def getPosition(self):
        return self.position.GetValue()/100.0

class controllerGUI(wx.Frame):
    TIMER_ID = 100

    def __init__(self, parent, debug = False):  
        wx.Frame.__init__(self, parent, -1, "Forktruck Controller GUI", style = wx.DEFAULT_FRAME_STYLE & ~ (wx.RESIZE_BORDER | wx.MAXIMIZE_BOX))
        sizer = wx.GridBagSizer(5,5)

        # Move Servos
        servo = wx.StaticBox(self, -1, 'Move Servos')
        servo.SetFont(wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.BOLD))
        servoBox = wx.StaticBoxSizer(servo,orient=wx.VERTICAL) 
        servoSizer = wx.GridBagSizer(5,5)

        self.servos = list()
        self.publishers = list()
        self.relaxers = list()

        joints = ["vplate_joint", "fork_joint", "tooth_joint", "camera_joint"]

        self.publishers.append(rospy.Publisher('joint1_position_controller/command', Float64))
        self.publishers.append(rospy.Publisher('joint2_position_controller/command', Float64))
        self.publishers.append(rospy.Publisher('joint3_position_controller/command', Float64))
        self.publishers.append(rospy.Publisher('joint4_position_controller/command', Float64))
        self.publishers.append(rospy.Publisher('joint5_position_controller/command', Float64))
        i = 0
        s = servoSlider(self, 0, 1.2, joints[i], i)
        servoSizer.Add(s.enabled,(i,0), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)   
        servoSizer.Add(s.position,(i,1), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)
        self.servos.append(s)
        i = 1
        s = servoSlider(self, 0, 2.85, joints[i], i)
        servoSizer.Add(s.enabled,(i,0), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)   
        servoSizer.Add(s.position,(i,1), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)
        self.servos.append(s)
        i = 2
        s = servoSlider(self, 0, 0.5, joints[i], i)
        servoSizer.Add(s.enabled,(i,0), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)   
        servoSizer.Add(s.position,(i,1), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)
        self.servos.append(s)

        i = 3
        s = servoSlider(self, -0.5, 2.35, joints[i], i)
        servoSizer.Add(s.enabled,(i,0), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)   
        servoSizer.Add(s.position,(i,1), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)
        self.servos.append(s)

        # add everything
        servoBox.Add(servoSizer) 
        sizer.Add(servoBox, (0,1), wx.GBSpan(1,1), wx.EXPAND|wx.TOP|wx.BOTTOM|wx.RIGHT,5)
        self.Bind(wx.EVT_CHECKBOX, self.enableSliders)
        # now we can subscribe
        rospy.Subscriber('joint_states', JointState, self.stateCb)

        # timer for output
        self.timer = wx.Timer(self, self.TIMER_ID)
        self.timer.Start(50)
        wx.EVT_CLOSE(self, self.onClose)
        wx.EVT_TIMER(self, self.TIMER_ID, self.onTimer)

        # bind the panel to the paint event
        #wx.EVT_PAINT(self, self.onPaint)
        self.dirty = 1
        #self.onPaint()

        self.SetSizerAndFit(sizer)
        self.Show(True)

    def onClose(self, event):
        self.timer.Stop()
        self.Destroy()

    def enableSliders(self, event):
        servo = event.GetId()
        if event.IsChecked(): 
            self.servos[servo].position.Enable()
        else:
            self.servos[servo].position.Disable()


    def stateCb(self, msg):        
        for servo in self.servos:
            if not servo.enabled.IsChecked():
                try:
                    idx = msg.name.index(servo.name)
                    servo.setPosition(msg.position[idx])
                except: 
                    pass



    def onTimer(self, event=None):
        # send joint updates
        if self.servos[0].enabled.IsChecked():
              d = Float64()
              d.data = self.servos[0].getPosition()
              self.publishers[0].publish(d)
        if self.servos[1].enabled.IsChecked():
              d = Float64()
              d.data = self.servos[1].getPosition()
              self.publishers[1].publish(d)
        if self.servos[2].enabled.IsChecked():
              d = Float64()
              d.data = self.servos[2].getPosition()
              self.publishers[2].publish(d)
              self.publishers[3].publish(d)
        if self.servos[3].enabled.IsChecked():
              d = Float64()
              d.data = self.servos[3].getPosition()
              self.publishers[4].publish(d)


if __name__ == '__main__':
    # initialize GUI
    rospy.init_node('controllerGUI')
    app = wx.PySimpleApp()
    frame = controllerGUI(None, True)
    app.MainLoop()

