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
from . import resource_rc

from behavior_actions.msg import AutoState, AutoMode
from geometry_msgs.msg import PoseWithCovarianceStamped
from imu_zero_msgs.srv import ImuZeroAngle
from behavior_actions.srv import DynamicPath
from base_trajectory_msgs.srv import GenerateSpline
import std_msgs.msg
import roslibpy
import os
from collections import OrderedDict

import logging
import scribble
import json

from python_qt_binding import QtCore

FILE_DIR = os.path.dirname(os.path.abspath(__file__))
MAP_PATH = os.path.abspath(os.path.join(FILE_DIR, '..', '..', 'resource', 'maps'))
MAP_CONFIGURATION = os.path.join(FILE_DIR, 'map_cfg.json')

IN_RANGE_IMG = QPixmap(":/images/GreenTarget.png")
NOT_IN_RANGE_IMG = QPixmap(":/images/RedTarget.png")

RED_STYLE = "background-color:#ff0000;"
GREEN_STYLE = "background-color:#5eff00;"

BALL_IMG_0 = QPixmap(":/images/0_balls.png")
BALL_IMG_1 = QPixmap(":/images/1_ball.png")
BALL_IMG_2 = QPixmap(":/images/2_balls.png")
BALL_IMG_3 = QPixmap(":/images/3_balls.png")
BALL_IMG_4 = QPixmap(":/images/4_balls.png")
BALL_IMG_5 = QPixmap(":/images/5_balls.png")
BALL_IMG_5PLUS = QPixmap(":/images/more_than_5_balls.png")

IMAGE_BL_X_IN_METERS = -0.4572
IMAGE_BL_Y_IN_METERS = -0.4572
METERS_PER_PIXEL = 0.00762

class Dashboard(Plugin):
    autoStateSignal = QtCore.pyqtSignal(int)
    nBallsSignal = QtCore.pyqtSignal(int)
    shooterInRangeSignal = QtCore.pyqtSignal(bool)
    turretInRangeSignal = QtCore.pyqtSignal(bool)

    msg_data = "default"
    def __init__(self, context):
        super(Dashboard, self).__init__(context)

        rospy.loginfo('Starting the Dashboard. Welcome :)')

        #Start client -rosbridge
        self.client = roslibpy.Ros(host='10.9.0.8', port=5803)
        self.client.run()

        # Give QObjects reasonable names
        self.setObjectName('Dashboard')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true", dest="quiet", help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        # if not args.quiet:
        #     print 'arguments: ', args
        #     print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_dashboard'), 'resource', 'DashboardAtHome.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('DashboardUi')


        self.draw_pad = scribble.ScribbleArea()
        self._widget.verticalLayout_7.insertWidget(0, self.draw_pad)

        # Load the map configurations
        with open(MAP_CONFIGURATION, 'r') as f:
            self.map_cfg = json.load(f, object_pairs_hook=OrderedDict)

        for path_name, map_cfg_path in self.map_cfg.items():
            if map_cfg_path:
                self.map_cfg[path_name] = os.path.join(MAP_PATH, map_cfg_path)

        for path_cfg in self.map_cfg:
            self._widget.path_combobox.addItem(path_cfg)
        self.set_path_plan(0)


        # Set up signal-slot connections
        self._widget.set_imu_angle_button.clicked.connect(self.setImuAngle)
        self._widget.imu_angle.valueChanged.connect(self.imuAngleChanged)

        self._widget.auto_wall_dist_button.clicked.connect(self.setAutoWallDist)
        self._widget.auto_wall_dist.valueChanged.connect(self.autoWallDistChanged)

        self._widget.ball_reset_button.clicked.connect(self.resetBallCount)
        self._widget.ball_reset_count.valueChanged.connect(self.resetBallChanged)

        self._widget.path_combobox.currentIndexChanged.connect(self.set_path_plan)
        self._widget.teleop_draw_check.stateChanged.connect(self.teleop_box_checked)

        self._widget.clear_btn.released.connect(self.draw_pad.reloadImage)
        self._widget.execute_path_btn.released.connect(self.start_execute_path)

        # Add buttons for auto modes
        v_layout = self._widget.auto_mode_v_layout #vertical layout storing the buttons
        self.auto_mode_button_group = QButtonGroup(self._widget) # needs to be a member variable so the publisher can access it to see which auto mode was selected


        # Search for auto_mode config items
        mode_idx = 0
        while rospy.has_param("/auto/auto_mode_{0:d}".format(mode_idx + 1)):
            auto_sequence = rospy.get_param("/auto/auto_mode_" + str(mode_idx + 1))

            new_auto_mode = QWidget()
            new_h_layout = QHBoxLayout()
            new_h_layout.setContentsMargins(0, 0, 0, 0)

            new_button = QRadioButton("Mode " + str(mode_idx + 1))
            new_button.setStyleSheet("font-weight: bold")
            self.auto_mode_button_group.addButton(new_button, mode_idx + 1) #  Second arg is the button's id
            new_h_layout.addWidget( new_button )

            new_h_layout.addWidget( QLabel(", ".join(auto_sequence)) )

            hSpacer = QSpacerItem(0, 0, QSizePolicy.Expanding, QSizePolicy.Minimum)
            new_h_layout.addItem(hSpacer)

            new_auto_mode.setLayout( new_h_layout )
            v_layout.addWidget(new_auto_mode)

            mode_idx += 1

        if mode_idx == 0:
            v_layout.addWidget( QLabel("No auto modes found") )
        else:
            rospy.loginfo("Auto modes found: {0:d}".format(mode_idx - 1))

        # auto state stuff
        self.autoState = 0
        self.displayAutoState() #display initial auto state

        # publish thread
        publish_thread = Thread(target=self.publish_thread) #args=(self,))
        publish_thread.start()


        self.n_balls = -1 #don't know n balls at first

        #in range stuff
        self.shooter_in_range = False
        self.turret_in_range = False
        self._widget.in_range_display.setPixmap(NOT_IN_RANGE_IMG)

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

        pf_location_listener = roslibpy.Topic(self.client, '/predicted_pose', 'pf_localization/pf_pose')
        self.pf_pose_sub = pf_location_listener.subscribe(self.robotPoseCallback)

        self.autoStateSignal.connect(self.autoStateSlot)
        self.nBallsSignal.connect(self.nBallsSlot)
        self.shooterInRangeSignal.connect(self.shooterInRangeSlot)
        self.turretInRangeSignal.connect(self.turretInRangeSlot)

        self.draw_pad.enableDrawing(False)
        rospy.loginfo('Dashboard loaded')


    def set_path_plan(self, idx):
        path_text = self._widget.path_combobox.currentText()
        map_path = self.map_cfg[path_text]

        if not map_path:
            rospy.loginfo('No map specified for path: {0:s}. Clearing draw area'.format(path_text))
            self.draw_pad.clearImage()
            self.draw_pad.unsetImage()
            return

        if not os.path.exists(map_path):
            rospy.loginfo('Unable to find map: {1:s} for path: {0:s}. Clearing draw area'.format(path_text, map_path))
            self.draw_pad.clearImage()
            self.draw_pad.unsetImage()
            return

        img_load = self.draw_pad.openImage(map_path)
        if img_load:
            rospy.loginfo('Successfully loaded map for path: {0:s}. Map file: {1:s}'.format(path_text, map_path))
        else:
            rospy.loginfo('Unable to load map image for path: {0:s} Map file: {1:s}'.format(path_text, map_path))


    def start_execute_path(self):
        rospy.loginfo('Execute path')
        coords = self.draw_pad.GetWorldCoordsRobotCentric()
        print(f'Map centric coordinates: {self.draw_pad.GetWorldCoords()}')
        print(f'Robot centric coordinates: {coords}')

        # Note the first point must always be (0, 0, 0) so modify it manually.
        coords[0] = scribble.Point(coords[0].x, coords[0].y)

        points = [
            {
                'positions': [pt.x, pt.y, 0],
                'velocities': [],
                'accelerations': [],
                'effort': [],
                'time_from_start': {'secs': 0, 'nsecs': 0}
            }
            for pt in coords
        ]

        path_offset_limits = [{
            'min_x': 0.0, 'max_x': 0.0,
            'min_y': 0.0, 'max_y': 0.0
            } for pt in coords
        ]

        msg = {
            # 'header': {'frame_id': 'map'},
            'points': points,
            'point_frame_id': [],
            'path_offset_limit': path_offset_limits,
            'optimize_final_velocity': False
        }

        self.setRobotPath(msg)

    def setRobotPath(self, msg):

        rospy.loginfo("Set robot path")

        try:
            # service = roslibpy.Service(self.client,'/base_trajectory/spline_gen', GenerateSpline)
            service = roslibpy.Service(self.client,'/path_follower/base_trajectory/spline_gen', msg)

            #Service Request-rosbridge
            request = roslibpy.ServiceRequest(msg)
            result = service.call(request)
            self.draw_pad.drawSplinePath(result['path']['poses'])

            rospy.loginfo("Successfully called spline generation")

            dynamic_path_client = {
                'path_name': self._widget.path_combobox.currentText() + ' Teleop',
                'dynamic_path': result['path']
            }
            # self.callExecutePath(result['path'])

            rospy.loginfo("Finished setting robot path")

        except (rospy.ServiceException, rospy.ROSException) as e: # the second exception happens if the wait for service times out
            self.errorPopup("Path generation error", e)


    def callExecutePath(self, path):
        rospy.loginfo("Begin call robot execution of path")

        try:
            # service = roslibpy.Service(self.client,'/base_trajectory/spline_gen', GenerateSpline)
            service = roslibpy.Service(self.client,'/auto/dynamic_path', path)

            #Service Request-rosbridge
            request = roslibpy.ServiceRequest(msg)
            result = service.call(path)

            rospy.loginfo("Successfully called dynamic path")

        except (rospy.ServiceException, rospy.ROSException) as e: # the second exception happens if the wait for service times out
            self.errorPopup("Path generation error", e)

        rospy.loginfo("Finished calling robot path execution")


    def teleop_box_checked(self, state):

        if state == QtCore.Qt.Checked:
            rospy.loginfo('Enabling Teleop mode. Drawing enabled')
            self.draw_pad.enableDrawing(True)
        else:
            rospy.loginfo('Enabling Auto mode. Drawing disabled')
            self.draw_pad.enableDrawing(False)


    def robotPoseCallback(self, msg):
        self.draw_pad.setRobotPosition(msg['x'], msg['y'])

    def autoStateCallback(self, msg):
        self.autoStateSignal.emit(int(msg['id']))

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
        self.nBallsSignal.emit(int(msg['data']))

    def nBallsSlot(self, state):

        if self.n_balls == state:
            return

        self.n_balls = state
        display = self._widget.n_balls_display

        if state == 0:
            display.setPixmap(BALL_IMG_0)
        elif state == 1:
            display.setPixmap(BALL_IMG_1)
        elif state == 2:
            display.setPixmap(BALL_IMG_2)
        elif state == 3:
            display.setPixmap(BALL_IMG_3)
        elif state == 4:
            display.setPixmap(BALL_IMG_4)
        elif state == 5:
            display.setPixmap(BALL_IMG_5)
        elif state > 5:
            display.setPixmap(BALL_IMG_5PLUS)
        else:
            display.setText("Couldn't read # balls")

    def shooterInRangeCallback(self, msg):
        self.shooterInRangeSignal.emit(bool(msg['data']))

    def shooterInRangeSlot(self, in_range):
        self.shooter_in_range = in_range #set here so that it's set synchronously with the other slots

    def turretInRangeCallback(self, msg):
        self.turretInRangeSignal.emit(bool(msg['data']))
        self.updateInRange()

    def turretInRangeSlot(self, in_range):
        self.turret_in_range = in_range #set here so that it's set synchronously with the other slots
        self.updateInRange()

    def updateInRange(self):
        display = self._widget.in_range_display
        if(self.shooter_in_range and self.turret_in_range):
            display.setPixmap(IN_RANGE_IMG)
        else:
            display.setPixmap(NOT_IN_RANGE_IMG)


    def setImuAngle(self):

        angle = self._widget.imu_angle.value() # imu_angle is the text field (doublespinbox) that the user can edit to change the navx angle, defaulting to zero
        rospy.loginfo("Set IMU angle: {0:0.1f}".format(angle))

        try:
            service = roslibpy.Service(self.client,'/imu/set_imu_zero',ImuZeroAngle)

            #Service Request-rosbridge
            request = roslibpy.ServiceRequest()
            result = service.call(request)

            # change button to green color to indicate that the service call went through
            self._widget.set_imu_angle_button.setStyleSheet(GREEN_STYLE)

        except (rospy.ServiceException, rospy.ROSException) as e: # the second exception happens if the wait for service times out
            self.errorPopup("Imu Set Angle Error", e)

        rospy.loginfo("Finished setting imu")


    def imuAngleChanged(self):
        # change button to red color if someone fiddled with the angle input, to indicate that input wasn't set yet
        self._widget.set_imu_angle_button.setStyleSheet(RED_STYLE)


    def setAutoWallDist(self):
        distance = self._widget.auto_wall_dist.value()
        rospy.loginfo("Set auto wall distance: {0:0.2f}".format(distance))
        self._widget.auto_wall_dist_button.setStyleSheet(GREEN_STYLE)

        rospy.loginfo("Finished setting auto wall distance")


    def autoWallDistChanged(self):
        self._widget.auto_wall_dist_button.setStyleSheet(RED_STYLE)


    def resetBallCount(self):
        nballs = self._widget.ball_reset_count.value()
        rospy.loginfo("Manually reset ball count: {0:d} balls".format(nballs))

        # call the service
        try:
            rospy.wait_for_service("/reset_ball", 1) #timeout in sec, TODO maybe put in config file?
            caller = rospy.ServiceProxy("/reset_ball", resetBallSrv)
            caller(nballs)
            # change button to green color to indicate that the service call went through
            self._widget.set_imu_angle_button.setStyleSheet(GREEN_STYLE)

        except (rospy.ServiceException, rospy.ROSException) as e: # the second exception happens if the wait for service times out
            self.errorPopup("Reset ball count Error", e)

        rospy.loginfo("Finished resetting ball count")


    def resetBallChanged(self):
        self._widget.ball_reset_button.setStyleSheet(RED_STYLE)


    def errorPopup(self, title, e):
        msg_box = QMessageBox()
        msg_box.setWindowTitle(title)
        msg_box.setIcon(QMessageBox.Warning)
        msg_box.setText("%s"%e)
        msg_box.exec_()

        rospy.logerror(e)


    #Publisher -> fake Auto States
    def publish_thread(self):

        pub = roslibpy.Topic(self.client, '/auto/auto_mode', 'behavior_actions/AutoMode')
        pub.advertise()

        enable_in_teleop_pub = roslibpy.Topic(self.client, '/enable_auto_in_teleop', 'std_msgs/Bool')
        enable_in_teleop_pub.advertise()

        r = rospy.Rate(10) # 10 Hz


        while self.client.is_connected:

            pub.publish(roslibpy.Message(
                {
                    'auto_mode': self.auto_mode_button_group.checkedId()
                }
            ))

            enable_in_teleop_pub.publish(roslibpy.Message(
                {
                    'data': self._widget.enable_in_teleop_box.checkState() == QtCore.Qt.Checked
                }
            ))

            r.sleep()


    def shutdown_plugin(self):

        rospy.loginfo('Begin shutdown sequence')

        if self.auto_state_sub is not None:
            self.auto_state_sub.unregister()

        if self.n_balls_sub is not None:
            self.n_balls_sub.unregister()

        if self.shooter_in_range_sub is not None:
            self.shooter_in_range_sub.unregister()

        if self.turret_in_range_sub is not None:
            self.turret_in_range_sub.unregister()

        self.client.close()

        rospy.loginfo('All done shutting down. Bye!')


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
