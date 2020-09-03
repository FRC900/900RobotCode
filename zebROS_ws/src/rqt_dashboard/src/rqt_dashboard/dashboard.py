import os
import rospy
import rospkg
from threading import Lock, Thread

import sys

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QGraphicsView, QPushButton, QRadioButton, QMessageBox, QHBoxLayout, QLabel, QButtonGroup, QSpacerItem, QSizePolicy
from python_qt_binding.QtCore import QCoreApplication
from python_qt_binding.QtGui import QPixmap
import resource_rc

from behavior_actions.msg import AutoState, AutoMode
from imu_zero.srv import ImuZeroAngle
from behavior_actions.srv import resetBallSrv
import std_msgs.msg
import roslibpy

from python_qt_binding import QtCore

class Dashboard(Plugin):
    autoStateSignal = QtCore.pyqtSignal(int)
    nBallsSignal = QtCore.pyqtSignal(int)
    shooterInRangeSignal = QtCore.pyqtSignal(bool)
    turretInRangeSignal = QtCore.pyqtSignal(bool)

    msg_data = "default"
    def __init__(self, context):
        #Start client -rosbridge
        self.client = roslibpy.Ros(host='localhost', port=5803)
        self.client.run()
        super(Dashboard, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Dashboard')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_dashboard'), 'resource', 'Dashboard.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('DashboardUi')


        # Set up signal-slot connections
        self._widget.set_imu_angle_button.clicked.connect(self.setImuAngle)
        self._widget.imu_angle.valueChanged.connect(self.imuAngleChanged)
        
        self._widget.auto_wall_dist_button.clicked.connect(self.setAutoWallDist)
        self._widget.auto_wall_dist.valueChanged.connect(self.autoWallDistChanged)
        
        self._widget.ball_reset_button.clicked.connect(self.resetBallCount)
        self._widget.ball_reset_count.valueChanged.connect(self.resetBallChanged)
        
        # Add buttons for auto modes
        v_layout = self._widget.auto_mode_v_layout #vertical layout storing the buttons
        self.auto_mode_button_group = QButtonGroup(self._widget) # needs to be a member variable so the publisher can access it to see which auto mode was selected
        # Search for auto_mode config items
        for i in range(1,100): # loop will exit when can't find the next auto mode, so really only a while loop needed, but exiting at 100 will prevent infinite looping
            if rospy.has_param("/auto/auto_mode_" + str(i)):
                auto_sequence = rospy.get_param("/auto/auto_mode_" + str(i))
               
                new_auto_mode = QWidget()
                new_h_layout = QHBoxLayout()
                new_h_layout.setContentsMargins(0,0,0,0)

                new_button = QRadioButton("Mode " + str(i))
                new_button.setStyleSheet("font-weight: bold") 
                self.auto_mode_button_group.addButton(new_button, i) #second arg is the button's id
                new_h_layout.addWidget( new_button )
                
                new_h_layout.addWidget( QLabel(", ".join(auto_sequence)) )

                hSpacer = QSpacerItem(0, 0, QSizePolicy.Expanding, QSizePolicy.Minimum)
                new_h_layout.addItem(hSpacer)

                new_auto_mode.setLayout( new_h_layout )
                v_layout.addWidget(new_auto_mode)
            else:
                print(str(i-1) + " auto modes found.")
                # if no auto modes found, inform the user with a label
                if (i-1) == 0:
                    v_layout.addWidget( QLabel("No auto modes found") )
                break #break out of for loop searching for auto modes
            
        # auto state stuff
        self.autoState = 0
        self.displayAutoState() #display initial auto state

        # publish thread
        publish_thread = Thread(target=self.publish_thread) #args=(self,))
        publish_thread.start()

        # number balls display
        self.zero_balls = QPixmap(":/images/0_balls.png")
        self.one_ball = QPixmap(":/images/1_ball.png")
        self.two_balls = QPixmap(":/images/2_balls.png")
        self.three_balls = QPixmap(":/images/3_balls.png")
        self.four_balls = QPixmap(":/images/4_balls.png")
        self.five_balls = QPixmap(":/images/5_balls.png")
        self.more_than_five_balls = QPixmap(":/images/more_than_5_balls.png")
        
        self.n_balls = -1 #don't know n balls at first 

        #in range stuff
        self.shooter_in_range = False
        self.turret_in_range = False
        self.in_range_pixmap = QPixmap(":/images/GreenTarget.png")
        self.not_in_range_pixmap = QPixmap(":/images/RedTarget.png")
        self._widget.in_range_display.setPixmap(self.not_in_range_pixmap)

        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        #initialize subscribers last, so that any callbacks they execute won't interfere with init
        auto_state_listener = roslibpy.Topic(self.client, '/auto/auto_state', 'behavior_actions/AutoState')
        self.auto_state_sub = auto_state_listener.subscribe(self.autoStateCallback)

        n_balls_listener = roslibpy.Topic(self.client,'/num_powercells','std_msgs/UInt8')
        self.n_balls_sub = n_balls_listener.subscribe(self.nBallsCallback)

        shooter_in_range_listener = roslibpy.Topic(self.client, '/shooter/shooter_in_range', std_msgs.msg.Bool)
        self.shooter_in_range_sub = shooter_in_range_listener.subscribe(self.shooterInRangeCallback)

        turret_in_range_listener = roslibpy.Topic(self.client, '/align_to_shoot/turret_in_range', std_msgs.msg.Bool)
        self.turret_in_range_sub = turret_in_range_listener.subscribe(self.turretInRangeCallback)

        self.autoStateSignal.connect(self.autoStateSlot)
        self.nBallsSignal.connect(self.nBallsSlot)
        self.shooterInRangeSignal.connect(self.shooterInRangeSlot)
        self.turretInRangeSignal.connect(self.turretInRangeSlot)


    def autoStateCallback(self, msg):
        self.autoStateSignal.emit(int(msg.id));

    def autoStateSlot(self, state):
        #self.lock.acquire()
        if(self.autoState != state):
            self.autoState = state
            self.displayAutoState()
    
    def displayAutoState(self):
        if self.autoState == 0:
            self._widget.auto_state_display.setText("Not ready")
            self._widget.auto_state_display.setStyleSheet("background-color:#ff5555;")
        elif self.autoState == 1: 
            self._widget.auto_state_display.setText("Ready, waiting for auto period")
            self._widget.auto_state_display.setStyleSheet("background-color:#ffffff;") 
        elif self.autoState == 2: 
            self._widget.auto_state_display.setText("Running")
            self._widget.auto_state_display.setStyleSheet("background-color:#ffff00")       
        elif self.autoState == 3: 
            self._widget.auto_state_display.setText("Finished")
            self._widget.auto_state_display.setStyleSheet("background-color:#00ff00;")
	elif self.autoState == 4:
	    self._widget.auto_state_display.setText("Error")
            self._widget.auto_state_display.setStyleSheet("background-color:#ff5555;")


    def nBallsCallback(self, msg):
        self.nBallsSignal.emit(int(msg.data))

    def nBallsSlot(self, state):
        if(self.n_balls != state):
            self.n_balls = state
            display = self._widget.n_balls_display
            
            if state == 0:
                display.setPixmap(self.zero_balls)
            elif state == 1:
                display.setPixmap(self.one_ball)
            elif state == 2:
                display.setPixmap(self.two_balls)
            elif state == 3:
                display.setPixmap(self.three_balls)
            elif state == 4:
                display.setPixmap(self.four_balls)
            elif state == 5:
                display.setPixmap(self.five_balls)
            elif state > 5:
                display.setPixmap(self.more_than_five_balls)
            else:
                display.setText("Couldn't read # balls")

    def shooterInRangeCallback(self, msg):
        self.shooterInRangeSignal.emit(bool(msg.data))

    def shooterInRangeSlot(self, in_range):
        self.shooter_in_range = in_range #set here so that it's set synchronously with the other slots

    def turretInRangeCallback(self, msg):
        self.turretInRangeSignal.emit(bool(msg.data))
        self.updateInRange()

    def turretInRangeSlot(self, in_range):
        self.turret_in_range = in_range #set here so that it's set synchronously with the other slots
        self.updateInRange()

    def updateInRange(self):
        display = self._widget.in_range_display
        if(self.shooter_in_range and self.turret_in_range):
            display.setPixmap(self.in_range_pixmap)
        else:
            display.setPixmap(self.not_in_range_pixmap)


    def setImuAngle(self):
        print("setting imu")
        #self.lock.acquire()
        angle = self._widget.imu_angle.value() # imu_angle is the text field (doublespinbox) that the user can edit to change the navx angle, defaulting to zero
        
        # call the service
        try:
            service = roslibpy.Service(self.client,'/imu/set_imu_zero',ImuZeroAngle)
           # rospy.wait_for_service("/imu/set_imu_zero", 1) # timeout in sec, TODO maybe put in config file?
            #TODO remove print
            #Service Request-rosbridge
            request = roslibpy.ServiceRequest()
            result = service.call(request)
            #result(angle)
            # change button to green color to indicate that the service call went through
            self._widget.set_imu_angle_button.setStyleSheet("background-color:#5eff00;")

        except (rospy.ServiceException, rospy.ROSException) as e: # the second exception happens if the wait for service times out
            self.errorPopup("Imu Set Angle Error", e)
        #self.lock.release()
        print("finished setting imu")

    def imuAngleChanged(self):
        #self.lock.acquire()
        # change button to red color if someone fiddled with the angle input, to indicate that input wasn't set yet
        self._widget.set_imu_angle_button
        self._widget.set_imu_angle_button.setStyleSheet("background-color:#ff0000;")
        #self.lock.release()


    def setAutoWallDist(self):
        print("setting auto wall distance")
        #self.lock.acquire()
        distance = self._widget.auto_wall_dist.value()
        
        self._widget.auto_wall_dist_button.setStyleSheet("background-color:#5eff00;")

        print("finished setting auto wall distance")

    def autoWallDistChanged(self):
        self._widget.auto_wall_dist_button.setStyleSheet("background-color:#ff0000;")
    
    def resetBallCount(self):
        print("manually reset ball count")
        #self.lock.acquire()
        nballs = self._widget.ball_reset_count.value() 
        
        # call the service
        try:
            rospy.wait_for_service("/reset_ball", 1) #timeout in sec, TODO maybe put in config file?
            caller = rospy.ServiceProxy("/reset_ball", resetBallSrv)
            caller(nballs)
            # change button to green color to indicate that the service call went through
            self._widget.set_imu_angle_button.setStyleSheet("background-color:##5eff00;")

        except (rospy.ServiceException, rospy.ROSException) as e: # the second exception happens if the wait for service times out
            self.errorPopup("Reset ball count Error", e)
        #self.lock.release()
        print("finished resetting ball count")

    def resetBallChanged(self):
        self._widget.ball_reset_button.setStyleSheet("background-color:#ff0000;")


    def errorPopup(self, title, e):
        msg_box = QMessageBox()
        msg_box.setWindowTitle(title)
        msg_box.setIcon(QMessageBox.Warning)
        msg_box.setText("%s"%e)
        msg_box.exec_()

    #Publisher -> fake Auto States
    def publish_thread(self):

        pub = roslibpy.Topic(self.client, '/auto/auto_mode', 'behavior_actions/AutoMode')
        r = rospy.Rate(10) # 10hz
        pub.advertise()
        while self.client.is_connected:

            pub.publish(roslibpy.Message(
                {
                    'auto_mode': self.auto_mode_button_group.checkedId()
                }
            ))
            r.sleep()

    def shutdown_plugin(self):
        self.auto_state_sub.unregister()
        self.n_balls_sub.unregister()
        self.shooter_in_range_sub.unregister()
        self.turret_in_range_sub.unregister()
        self.client.close()


    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass
    

    def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
        pass

