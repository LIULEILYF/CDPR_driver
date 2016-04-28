#!/usr/bin/python

# based on joint_state_publisher by David Lu!!
#
#This is a simple GUI for publishing a single TF transform.
#    Maintainer: Martin Guenther <mguenthe AT uni-osnabrueck DOT de>
#    Author: Martin Guenther
#    License: BSD
#    Source: git https://github.com/uos/uos_tools.git (branch: indigo)


import roslib; 
import rospy
import wx
import tf
from math import pi
from threading import Thread

RANGE = 1000000

class TfPublisher():
    def __init__(self):
        self.parent_frame = 'world'
        self.child_frame = 'desired_platform'

        self.elements = {}
        self.element_list = [] # for maintaining the original order of the elements

        for name in ['x', 'y', 'z']:
            element = {'min':-1, 'max':1, 'zero':0.0, 'value':0.0}
            self.elements[name] = element
            self.element_list.append(name)

        for name in ['roll', 'pitch', 'yaw']:
            element = {'min':-pi, 'max':pi, 'zero':0.0, 'value':0.0}
            self.elements[name] = element
            self.element_list.append(name)

    def loop(self):
        hz = rospy.get_param("~rate", 10) # 10hz
        r = rospy.Rate(hz) 

        # Publish TF messages
        while not rospy.is_shutdown():
            br = tf.TransformBroadcaster()
            br.sendTransform((self.elements['x']['value']/1000., self.elements['y']['value']/1000., self.elements['z']['value']/1000.),
                             tf.transformations.quaternion_from_euler(
                                self.elements['roll']['value']/1000.,
                                self.elements['pitch']['value']/1000.,
                                self.elements['yaw']['value']/1000.),
                             rospy.Time.now(),
                             self.child_frame,
                             self.parent_frame)

            r.sleep()

class TfPublisherGui(wx.Frame):
    def __init__(self, title, tfp):
        wx.Frame.__init__(self, None, -1, title, (-1,-1));
        self.tfp = tfp
        self.element_map = {}
        panel = wx.Panel(self, wx.ID_ANY);
        box = wx.BoxSizer(wx.VERTICAL)
        font = wx.Font(9, wx.SWISS, wx.NORMAL, wx.BOLD)
        
        ### Sliders ###
        for name in self.tfp.element_list:
            element = self.tfp.elements[name]

            if element['min'] == element['max']:
                continue

            row = wx.GridSizer(1,2)
            label = wx.StaticText(panel, -1, name)
            label.SetFont(font)
            row.Add(label, 1, wx.ALIGN_CENTER_VERTICAL)

            display = wx.TextCtrl (panel, value=str(0), 
                        style=wx.TE_READONLY | wx.ALIGN_RIGHT)

            row.Add(display, flag= wx.ALIGN_RIGHT| wx.ALIGN_CENTER_VERTICAL)
            box.Add(row, 1, wx.EXPAND)
            slider = wx.Slider(panel, -1, RANGE/2, 0, RANGE, 
                        style= wx.SL_AUTOTICKS | wx.SL_HORIZONTAL)
            slider.SetFont(font)
            box.Add(slider, 1, wx.EXPAND)

            self.element_map[name] = {'slidervalue':0, 'display':display, 
                                    'slider':slider, 'element':element}

        ### Buttons ###
        self.ctrbutton = wx.Button(panel, 1, 'Center')
        self.Bind(wx.EVT_SLIDER, self.sliderUpdate)
        
        wx.EVT_BUTTON(self, 1, self.center_event)

        box.Add(self.ctrbutton, 0, wx.EXPAND)
        
        panel.SetSizer(box)
        self.center()
        box.Fit(self)
        self.update_values()


    def update_values(self):
        for (name,element_info) in self.element_map.items():
            purevalue = element_info['slidervalue']
            element = element_info['element']
            value = self.sliderToValue(purevalue, element)
            element['value'] = value
            element_info['slider'].SetValue(purevalue)
            element_info['display'].SetValue("%.2f"%value)

    def center_event(self, event):
        self.center()

    def center(self):
        rospy.loginfo("Centering")
        for (name,element_info) in self.element_map.items():
            element = element_info['element']
            element_info['slidervalue'] = self.valueToSlider(element['zero'], element)
        self.update_values()

    def sliderUpdate(self, event):
        for (name,element_info) in self.element_map.items():
            element_info['slidervalue'] = element_info['slider'].GetValue()
        self.update_values()

    def valueToSlider(self, value, element):
        return (value - element['min']) * float(RANGE) / (element['max'] - element['min'])
        
    def sliderToValue(self, slider, element):
        pctvalue = slider / float(RANGE)
        return element['min'] + (element['max']-element['min']) * pctvalue


if __name__ == '__main__':
    try:
        rospy.init_node('tf_publisher_gui')
        tfp = TfPublisher()

        app = wx.App()
        gui = TfPublisherGui("Delta TF Publisher", tfp)
        gui.Show()
        Thread(target=app.MainLoop).start()
        
        tfp.loop()
        
    except rospy.ROSInterruptException: pass
