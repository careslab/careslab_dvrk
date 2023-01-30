import os
import rospy
import rospkg
import signal
import pexpect
import time
import subprocess
from std_msgs.msg import String, Empty, Bool

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QMessageBox


class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        #if not args.quiet:
            #print 'arguments: ', args
            #print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('dvrk_cares_rqt_plugin'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self._widget.powerOnButton.pressed.connect(self._on_powerOnButton_pressed)
        self._widget.powerOffButton.pressed.connect(self._on_powerOffButton_pressed)
        self._widget.homeButton.pressed.connect(self._on_homeButton_pressed)
        self._widget.resetButton.pressed.connect(self._on_resetButton_pressed)

        self._widget.autocameraRadioButton.pressed.connect(self._on_autocameraRadioButton_pressed)
        self._widget.clutchandMoveRadioButton.pressed.connect(self._on_clutchandMoveRadioButton_pressed)
        self._widget.joystickRadioButton.pressed.connect(self._on_joystickRadioButton_pressed)
        self._widget.voiceControlRadioButton.pressed.connect(self._on_voiceControlRadioButton_pressed)

        self._widget.startRecording.pressed.connect(self._on_startRecording_pressed)
        self._widget.stopRecording.pressed.connect(self._on_stopRecording_pressed)


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

    def set_all_control_algorithms_off(self):
        #rospy.Publisher('/assistant/teleop/run', Bool, latch=True, queue_size=1).publish(Bool(False))
        rospy.Publisher('/assistant/autocamera/run', Bool, latch=True, queue_size=1).publish(Bool(False))
        rospy.Publisher('/assistant/clutch_and_move/run', Bool, latch=True, queue_size=1).publish(Bool(False))
        rospy.Publisher('/assistant/joystick/run', Bool, latch=True, queue_size=1).publish(Bool(False))
        rospy.Publisher('/assistant/oculus/run', Bool, latch=True, queue_size=1).publish(Bool(False))
        rospy.Publisher('/assistant/clutchless/run', Bool, latch=True, queue_size=1).publish(Bool(False))
        
    def _on_powerOnButton_pressed(self):
        rospy.Publisher('/assistant/home', Empty, latch=True, queue_size=1).publish(Empty())
        print("RAN POWER ON")
        self._widget.powerOnButton.setEnabled(False)
        self._widget.homeButton.setEnabled(True)
        self._widget.powerOffButton.setEnabled(True)
        self._widget.resetButton.setEnabled(True)
        self._widget.startRecording.setEnabled(True)
        self._widget.stopRecording.setEnabled(True)

    def _on_powerOffButton_pressed(self):
        rospy.Publisher('/assistant/power_off', Empty, latch=True, queue_size=1).publish(Empty())
        print("RAN POWER OFF")
        self._widget.powerOnButton.setEnabled(True)
        self._widget.homeButton.setEnabled(False)
        self._widget.powerOffButton.setEnabled(False)
        self._widget.resetButton.setEnabled(False)

        self._widget.autocameraRadioButton.setEnabled(False)
        self._widget.clutchandMoveRadioButton.setEnabled(False)
        self._widget.clutchlessSystemRadioButton.setEnabled(False)
        self._widget.joystickRadioButton.setEnabled(False)
        self._widget.oculusRadioButton.setEnabled(False)
        self._widget.voiceControlRadioButton.setEnabled(False)

    def _on_homeButton_pressed(self):
        rospy.Publisher('/assistant/teleop/run', Bool, latch=True, queue_size=1).publish(Bool(True))
        self._widget.autocameraRadioButton.setEnabled(True)
        self._widget.clutchandMoveRadioButton.setEnabled(True)
        self._widget.clutchlessSystemRadioButton.setEnabled(True)
        self._widget.joystickRadioButton.setEnabled(True)
        self._widget.oculusRadioButton.setEnabled(True)
        self._widget.voiceControlRadioButton.setEnabled(True)

    def _on_resetButton_pressed(self):
        rospy.Publisher('/assistant/reset', Empty, latch=True, queue_size=1).publish(Empty())
        print("RAN RESET")

    def _on_autocameraRadioButton_pressed(self):
        if not self._widget.autocameraRadioButton.isChecked():
            self.set_all_control_algorithms_off()
            rospy.Publisher('/assistant/autocamera/run', Bool, latch=True, queue_size=1).publish(Bool(True))
            msg = QMessageBox()
            msg.setText('running autocamera')
            retval = msg.exec_()
        self._widget.autocameraRadioButton.setChecked(True)

    def _on_clutchandMoveRadioButton_pressed(self):
        if not self._widget.clutchandMoveRadioButton.isChecked():
            self.set_all_control_algorithms_off()
            rospy.Publisher('/assistant/clutch_and_move/run', Bool, latch=True, queue_size=1).publish(Bool(True))
            msg = QMessageBox()
            msg.setText('running clutch and move')
            retval = msg.exec_()
        self._widget.clutchandMoveRadioButton.setChecked(True)

    def _on_joystickRadioButton_pressed(self):
        if not self._widget.joystickRadioButton.isChecked():
            self.set_all_control_algorithms_off()
            rospy.Publisher('/assistant/joystick/run', Bool, latch=True, queue_size=1).publish(Bool(True))
            msg = QMessageBox()
            msg.setText('running joystick')
            retval = msg.exec_()
        self._widget.joystickRadioButton.setChecked(True)
    
    def _on_voiceControlRadioButton_pressed(self):
        os.chdir('/home/cares/catkin_ws/src/careslab_dvrk/dvrk_voice/scripts')
        subprocess.call([ "gnome-terminal", "-x", "./run_voice.sh"])

    def _on_startRecording_pressed(self):
        self._widget.displayLabel.setText("Recording has started.")
        subject_value = str(self._widget.subjectValueBox.value())
        task_value = str(self._widget.taskValueBox.value())
        timestr = time.strftime("%Y%m%d-%H%M%S")
        directory = '/home/cares/catkin_ws/bagfiles/'
        folder_name = "Subject_" + subject_value + "/" + "Task_" + task_value
        folder_path = os.path.join(directory,folder_name)        
        os.makedirs(folder_path)
        dir = folder_path + "/"
        bag_name = "Recording_" + timestr
        record_command = """rosbag record /dvrk/MTML/state_joint_current /dvrk/MTMR/state_joint_current /dvrk/PSM1/state_joint_current /dvrk/PSM1/position_cartesian_current
        /dvrk/PSM2/state_joint_current /dvrk/PSM2/position_cartesian_current
        /dvrk/ECM/state_joint_current /dvrk/ECM/position_cartesian_current
        /dvrk/footpedals/clutch /dvrk/footpedals/camera /dvrk/footpedals/coag /joy /image_raw_left/compressed /image_raw_right/compressed --lz4 --duration=180 -O {}""".format(dir +bag_name + '.bag' )
        print(record_command)
        self._shellcmd = pexpect.spawn(record_command)
        self._widget.displayLabel.setText("Recording has started for Subject: "+ subject_value + " Task: " + task_value)
        

    def _on_stopRecording_pressed(self):
        self._shellcmd.sendcontrol('c')
        self._shellcmd.sendintr()
        self._shellcmd.close()
        print('recording finished')
        self._widget.displayLabel.setText("Recording has stopped.")




