#!/usr/bin/env python3

from cgitb import enable
import os
from sys import exit
import argparse
import rospy
import rospkg
import threading
from PyQt5.QtWidgets import QWidget, QCheckBox, QApplication, QHBoxLayout, QVBoxLayout, QLabel
from PyQt5 import QtCore, QtWidgets
#from PyQt5 import QtGui
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from behavior_actions.msg import AutoState
from frc_msgs.msg import MatchSpecificData
from talon_state_msgs.msg import TalonState
from talon_state_msgs.msg import TalonFXProState
from ros_control_boilerplate.srv import LineBreakSensors, set_limit_switch

class DriverStationSim(Plugin):
    auto_state_signal = QtCore.pyqtSignal(int)
    def _read_joint_param(self, param_key, where, talons, talonfxpros, ains, dins):
        if not rospy.has_param(param_key):
            return
        joint_param = rospy.get_param(param_key)
        for jp in joint_param:
            if ('local' in jp) and (jp['local'] == False):
                continue
            if ('local_hardware' in jp) and (jp['local_hardware'] == False):
                continue
            if ('can_talon' not in jp['type']) and (jp['type'] != 'talonfxpro') and (jp['type'] != 'analog_input') and (jp['type'] != 'digital_input'): 
               continue
            entry = {}
            entry['where'] = where
            entry['name'] = jp['name']
            if ('can_talon' in jp['type']):
                talons.append(entry)
            if ('talonfxpro' in jp['type']):
                talonfxpros.append(entry)
            if jp['type'] == 'analog_input':
                ains.append(entry)
            if jp['type'] == 'digital_input':
                dins.append(entry)

        #print talons
        #print ains
        #print dins

    def _talon_checkbox_handler(self, data = None):
        sender = self.sender()
        obj_name = sender.objectName()
        obj_state = sender.isChecked()
        other_obj_state = sender.other_button.isChecked()

        args = obj_name.split("%")
        if (args[1] == "F"): # Forward button checked
            forward = obj_state
            reverse = other_obj_state
        else:
            forward = other_obj_state
            reverse = obj_state

        try:
            limit_switch_service = rospy.ServiceProxy('/frcrobot_' + sender.where + '/' + args[2], set_limit_switch)
            #print "Calling service %s:  %d %s %d %d" %('/frcrobot_' + sender.where +'/set_limit_switch', 0, obj_name[:-2], forward, reverse)
            limit_switch_service(0, args[0], forward, reverse)
        except rospy.ServiceException as e:
            print(f"set_limit_switch service call failed: {e}")


    def _din_checkbox_handler(self, data = None):
        sender = self.sender()

        try:
            linebreak_service = rospy.ServiceProxy('/frcrobot_' + sender.where +'/linebreak_service_set', LineBreakSensors)
            #print "Calling service %s:  %d %s %d " %('/frcrobot_' + sender.where +'/linebreak_service_set', 0, sender.objectName(), sender.isChecked())
            linebreak_service(0, sender.objectName(), sender.isChecked())

        except rospy.ServiceException as e:
            print(f"linebreak_service call failed: {e}")


    def _auto_state_callback(self, msg):
        self.auto_state_signal.emit(int(msg.id))

    def _talon_state_callback(self, msg):
        # name + ": " + percent_output + "%, set to " + setpoint
        for i in range(len(msg.name)):
            self._widget.talon_state_widgets[i].setText(str(msg.name[i])+" (in "+str(msg.talon_mode[i])+" mode): "+str(round(float(msg.motor_output_percent[i]), 4))+"%, position: "+str(round(float(msg.position[i]), 4))+", speed: "+str(round(float(msg.speed[i]), 4)) +", setpoint: "+str(round(float(msg.set_point[i]), 4)))
            # self._widget.talon_state_widgets[i].adjustSize()

    def _talonfxpro_state_callback(self, msg):
        # name + ": " + percent_output + "%, set to " + setpoint
        for i in range(len(msg.name)):
            self._widget.talon_state_widgets[i + self.num_talons].setText(str(msg.name[i])+" (in "+str(msg.control_mode[i])+" mode): "+str(round(float(msg.duty_cycle[i]), 4))+"%, position: "+str(round(float(msg.position[i]), 4))+", speed: "+str(round(float(msg.velocity[i]), 4))+", control_output: "+str(round(float(msg.control_output[i]), 4)))
            # self._widget.talon_state_widgets[i].adjustSize()

    def auto_state_slot(self, state):
        if(self.auto_state != state):
            self.auto_state = state
            self.display_auto_state()


    def display_auto_state(self):
        if self.auto_state == 0:
            self._widget.auto_state_readback_text.setText("Not ready")
            self._widget.auto_state_readback_text.setStyleSheet("background-color:#ff5555;")
        elif self.auto_state == 1:
            self._widget.auto_state_readback_text.setText("Ready, waiting for auto period")
            self._widget.auto_state_readback_text.setStyleSheet("background-color:#ffffff;")
        elif self.auto_state == 2:
            self._widget.auto_state_readback_text.setText("Running")
            self._widget.auto_state_readback_text.setStyleSheet("background-color:#ffff00")
        elif self.auto_state == 3:
            self._widget.auto_state_readback_text.setText("Finished")
            self._widget.auto_state_readback_text.setStyleSheet("background-color:#00ff00;")
        elif self.auto_state == 4:
            self._widget.auto_state_readback_text.setText("Error")
            self._widget.auto_state_readback_text.setStyleSheet("background-color:#ff5555;")

    def get_alliance_location(self): # returns (allianceColor, driverStationLocation) -- for color, 0 = red and 1 = blue
        if self._widget.red1.isChecked():
            return 0, 1
        if self._widget.red2.isChecked():
            return 0, 2
        if self._widget.red3.isChecked():
            return 0, 3
        if self._widget.blue1.isChecked():
            return 1, 1
        if self._widget.blue2.isChecked():
            return 1, 2
        if self._widget.blue3.isChecked():
            return 1, 3

    def __init__(self, context):
        super(DriverStationSim, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('DriverStationSim')
        self.stop_pub = False

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_driver_station_sim'), 'resource', 'driverStationSim.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('DriverStationSim')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.

        talons      = []
        talonfxpros = []
        dins        = []
        ains        = []
        self._read_joint_param('/frcrobot_rio/hardware_interface/joints', 'rio', talons, talonfxpros, ains, dins)
        self._read_joint_param('/frcrobot_jetson/hardware_interface/joints', 'jetson', talons, talonfxpros, ains, dins)

        #self._widget.talons_tab = QtWidgets.QWidget(self._widget)
        #self._widget.talons_tab.setObjectName("talons_tab")
        self._widget.sim_input_layout_widget = QtWidgets.QWidget(self._widget.tab_4)
        self._widget.sim_input_layout_widget.setObjectName("sim_input_layout_widget")
        self._widget.sim_input_horizontal_layout = QtWidgets.QHBoxLayout(self._widget.sim_input_layout_widget)
        self._widget.sim_input_horizontal_layout.setObjectName("sim_input_horizontal_layout")
        self._widget.talon_f_vertical_layout = QtWidgets.QVBoxLayout()
        self._widget.talon_f_vertical_layout.setObjectName("talon_f_vertical_layout")
        self._widget.talon_r_vertical_layout = QtWidgets.QVBoxLayout()
        self._widget.talon_r_vertical_layout.setObjectName("talon_r_vertical_layout")
        self._widget.talon_f_buttons = []
        self._widget.talon_r_buttons = []
        fh = 17 # Set fixed height to prevent ugly auto-spacing
        for i in range(len(talons)):
            self._widget.talon_f_buttons.append(QCheckBox("F", self._widget.sim_input_layout_widget))
            self._widget.talon_r_buttons.append(QCheckBox("R | " + talons[i]['name'], self._widget.sim_input_layout_widget))

            self._widget.talon_f_buttons[-1].setObjectName(talons[i]['name'] + "%F%set_limit_switch")
            self._widget.talon_f_buttons[-1].setFixedHeight(fh)
            self._widget.talon_f_buttons[-1].where = talons[i]['where']
            self._widget.talon_f_buttons[-1].other_button = self._widget.talon_r_buttons[i]
            self._widget.talon_f_buttons[-1].stateChanged.connect(self._talon_checkbox_handler)
            self._widget.talon_f_vertical_layout.addWidget(self._widget.talon_f_buttons[-1])
            self._widget.talon_r_buttons[-1].setObjectName(talons[i]['name'] + "%R%set_limit_switch")
            self._widget.talon_r_buttons[-1].setFixedHeight(fh)
            self._widget.talon_r_buttons[-1].where = talons[i]['where']
            self._widget.talon_r_buttons[-1].other_button = self._widget.talon_f_buttons[i]
            self._widget.talon_r_buttons[-1].stateChanged.connect(self._talon_checkbox_handler)
            self._widget.talon_r_vertical_layout.addWidget(self._widget.talon_r_buttons[-1])
        for i in range(len(talonfxpros)):
            self._widget.talon_f_buttons.append(QCheckBox("F", self._widget.sim_input_layout_widget))
            self._widget.talon_r_buttons.append(QCheckBox("R | " + talonfxpros[i]['name'], self._widget.sim_input_layout_widget))

            self._widget.talon_f_buttons[-1].setObjectName(talonfxpros[i]['name'] + "%F%set_talonfxpro_limit_switch")
            self._widget.talon_f_buttons[-1].setFixedHeight(fh)
            self._widget.talon_f_buttons[-1].where = talonfxpros[i]['where']
            self._widget.talon_f_buttons[-1].other_button = self._widget.talon_r_buttons[i]
            self._widget.talon_f_buttons[-1].stateChanged.connect(self._talon_checkbox_handler)
            self._widget.talon_f_vertical_layout.addWidget(self._widget.talon_f_buttons[-1])
            self._widget.talon_r_buttons[-1].setObjectName(talonfxpros[i]['name'] + "%R%set_talonfxpro_limit_switch")
            self._widget.talon_r_buttons[-1].setFixedHeight(fh)
            self._widget.talon_r_buttons[-1].where = talonfxpros[i]['where']
            self._widget.talon_r_buttons[-1].other_button = self._widget.talon_f_buttons[i]
            self._widget.talon_r_buttons[-1].stateChanged.connect(self._talon_checkbox_handler)
            self._widget.talon_r_vertical_layout.addWidget(self._widget.talon_r_buttons[i])
        self._widget.sim_input_horizontal_layout.addLayout(self._widget.talon_f_vertical_layout)
        self._widget.sim_input_horizontal_layout.addLayout(self._widget.talon_r_vertical_layout)

        self._widget.din_vertical_layout = QtWidgets.QVBoxLayout()
        self._widget.din_vertical_layout.setObjectName("din_vertical_layout")
        self._widget.din_buttons = []
        for i in range(len(dins)):
            self._widget.din_buttons.append(QCheckBox(dins[i]['name'], self._widget.sim_input_layout_widget))
            self._widget.din_buttons[i].setObjectName(dins[i]['name'])
            self._widget.din_buttons[i].setFixedHeight(fh)
            self._widget.din_buttons[i].where = dins[i]['where']
            self._widget.din_buttons[i].stateChanged.connect(self._din_checkbox_handler)
            self._widget.din_vertical_layout.addWidget(self._widget.din_buttons[i])
        self._widget.sim_input_horizontal_layout.addLayout(self._widget.din_vertical_layout)

        self._widget.ain_vertical_layout = QtWidgets.QVBoxLayout()
        self._widget.ain_vertical_layout.setObjectName("ain_vertical_layout")
        self._widget.ain_buttons = []
        for i in range(len(ains)):
            self._widget.ain_buttons.append(QDoubleSpinBox(ains[i]['name'], self._widget.sim_input_layout_widget))
            self._widget.ain_buttons[i].setObjectName(ains[i]['name'])
            self.delay_0.setDecimals(2)
            self.delay_0.setMaximum(12.0)
            self.delay_0.setMinimum(-12.0)
            self.delay_0.setSingleStep(0.1)
            self._widget.ain_buttons[i].setFixedHeight(fh)
            self._widget.ain_buttons[i].where = ains[i]['where']
            self._widget.ain_vertical_layout.addWidget(self._widget.ain_buttons[i])
            self._widget.sim_input_horizontal_layout.addLayout(self._widget.ain_vertical_layout)

            fh = 17 # Set fixed height to prevent ugly auto-spacing

        self._widget.talon_state_layout_widget = QtWidgets.QWidget(self._widget.tab)
        self._widget.talon_state_layout_widget.setObjectName("talon_state_layout_widget")
        self._widget.talon_state_vertical_layout = QtWidgets.QVBoxLayout(self._widget.talon_state_layout_widget)
        self._widget.talon_state_vertical_layout.setObjectName("talon_state_vertical_layout")
        # Name, mode, motor output percent, setpoint
        self._talon_states_sub = rospy.Subscriber("/frcrobot_jetson/talon_states", TalonState, self._talon_state_callback)
        self._talonfxpro_states_sub = rospy.Subscriber("/frcrobot_jetson/talonfxpro_states", TalonFXProState, self._talonfxpro_state_callback)
        self._widget.talon_state_widgets = []
        for i in range(len(talons)):
            self._widget.talon_state_widgets.append(QLabel("Talon" + " "*540))
            self._widget.talon_state_vertical_layout.addWidget(self._widget.talon_state_widgets[-1])
        self.num_talons = len(talons)
        for i in range(len(talonfxpros)):
            self._widget.talon_state_widgets.append(QLabel("TalonFXPro" + " "*540))
            self._widget.talon_state_vertical_layout.addWidget(self._widget.talon_state_widgets[-1])


        self.auto_state_signal.connect(self.auto_state_slot)
        self.auto_state_sub = rospy.Subscriber("/auto/auto_state", AutoState, self._auto_state_callback)
        self.auto_state = 0
        self.display_auto_state()

        self._widget.disable_button_2.setChecked(True)
        self._widget.teleop_button.setChecked(True)
        def pub_data(self):
            r = rospy.Rate(20)

            match_pub = rospy.Publisher("/frcrobot_rio/match_data_in", MatchSpecificData, queue_size=2)
            match_msg = MatchSpecificData()
            start_time = rospy.get_time()
            enable_last = False
            auto_last = False
            practice_last = False
            auto_duration = 0
            while not self.stop_pub and not rospy.is_shutdown():
                #Robot State Values
                enable = self._widget.enable_button_2.isChecked()
                disable = self._widget.disable_button_2.isChecked()
                auto = self._widget.auto_mode.isChecked()
                practice = self._widget.practice_button.isChecked()


                #Time Start and Restart Handling
                if(not enable_last and enable):
                    rospy.logwarn("enableLast")
                    start_time = rospy.get_time()
                if(not auto_last and auto and not practice):
                    rospy.logwarn("autoLast")
                    start_time = rospy.get_time()
                if(not practice_last and practice):
                    rospy.logwarn("practiceLast")
                    start_time = rospy.get_time()
                    auto_duration = 15 #TODO read from DS
                if(enable and practice):
                    if(rospy.get_time() < start_time + auto_duration):
                        auto = True
                        enable = True
                        disable = False
                    elif(rospy.get_time() >= start_time + auto_duration and rospy.get_time < start_time + 150):
                        auto = False
                        enable = True
                        disable = False
                    elif(rospy.get_time() >= start_time + 150):
                        auto = False
                        enable = False
                        disable = True


                if(enable):
                    time_diff = int(rospy.get_time()-start_time)
                    minutes = ((150-time_diff)/60)
                    seconds = ((149-time_diff)%60 + (1.0-(rospy.get_time()-start_time)%1))
                    time = str(int(minutes)) + ":" + str(round(seconds, 1))
                    if(not self._widget.timer_pause.isChecked()):
                        self._widget.time.setText(time[:7])
                    match_msg.matchTimeRemaining = 150-time_diff
                    match_msg.Disabled = False
                    match_msg.Enabled = True
                else:
                    match_msg.matchTimeRemaining = 0
                    match_msg.Disabled = True
                    match_msg.Enabled = False

                #Publish Data
                match_msg.header.stamp = rospy.Time.now()
                match_msg.gameSpecificData = [ord(c) for c in self._widget.game_specific_data.text()]
                match_msg.allianceColor, match_msg.driverStationLocation = self.get_alliance_location()
                match_msg.matchNumber = 1
                match_msg.Autonomous = auto
                match_msg.DSAttached = True

                # TODO - FMS attached
                # TODO - EStopped?

                enable_last = match_msg.Enabled
                auto_last = auto
                practice_last = practice

                match_pub.publish(match_msg)

                r.sleep()

            match_pub.unregister()
            sys.exit(0)

        self._load_thread = threading.Thread(target=pub_data, args=(self,))
        self._load_thread.start()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self.stop_pub = True
        self._load_thread.join(5)

        if self.auto_state_sub is not None:
            self.auto_state_sub.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def _parse_args(self, argv):
        parser = argparse.ArgumentParser(prog="rqt_driver_station_sim", add_help=False)
        DriverStationSim.add_arguments(parser)
        return parser.parse_args(argv)


    @staticmethod
    def add_arguments(parser):
        group = parser.add_argument_group('Options for rqt_driver_station_sim plugin')
        #No options exist yet

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
