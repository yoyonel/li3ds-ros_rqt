import os
import rospkg
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtCore import Slot
from PyQt4.QtCore import QTimer

# from std_msgs.msg import String
from velodyne_configuration.msg import VLP16_StatusMessage
from velodyne_configuration.msg import VLP16_DiagnosticsMessage


class LI3DSPlugin(Plugin):
    def __init__(self, context):
        super(LI3DSPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('LI3DSPlugin')
        rp = rospkg.RosPack()

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
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(rp.get_path('rqt_li3ds'), 'resource', 'LI3DSPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('LI3DSPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # ---------------------------------
        # SETTING des connections slots
        # ---------------------------------
        #        rospy.loginfo("Test log!")
        self._widget.pushButton_record_on.clicked[bool].connect(self.on_pushButton_record_on_clicked)
        self._widget.pushButton_record_off.clicked[bool].connect(self.on_pushButton_record_off_clicked)
        # ---------------------------------

        # ---------------------------------
        # SUBSCRIBERS
        # ---------------------------------
        # Status
        self._subscriber_status = rospy.Subscriber('/velodyne_status_server/status_pub',
                                                   VLP16_StatusMessage,
                                                   self.cb_status,
                                                   queue_size=10
                                                   )
        # Diagnostics
        self._subscriber_status = rospy.Subscriber('/velodyne_diagnostics_server/diagnostics_pub',
                                                   VLP16_DiagnosticsMessage,
                                                   self.cb_diagnostics,
                                                   queue_size=10
                                                   )
        # Settings
        # ---------------------------------
        self._VLP16Status = None
        self._VLP16Diagnostics = None

        # ---------------------------------
        # GUI: QT
        # ---------------------------------
        # Status
        self._update_GUI_status_timer = QTimer()
        self._update_GUI_status_timer.timeout.connect(self.on_update_GUI_status_timer)
        self._update_GUI_status_timer.setInterval(500)
        # Diagnostics
        self._update_GUI_diagnostics_timer = QTimer()
        self._update_GUI_diagnostics_timer.timeout.connect(self.on_update_GUI_diagnostics_timer)
        self._update_GUI_diagnostics_timer.setInterval(500)
        # Start QTimers
        self._update_GUI_status_timer.start()
        self._update_GUI_diagnostics_timer.start()

        self.map_top_qt_ros_msg = {
            # Top Board
            'HV': 'top_hv',
            'A/D TD': 'top_ad_temp',
            'Temp': 'top_lm20_temp',
            '5v': 'top_pwr_5v',
            '5v (Raw)': 'top_pwr_5v_raw',
            '2.5v': 'top_pwr_2_5v',
            '3.3v': 'top_pwr_3_3v',
            'VHV': 'vhv',
        }
        self.map_bot_qt_ros_msg = {
            # Bottom Board
            'I out': 'bot_i_out',
            '1.2v': 'bot_pwr_1_2v',
            'Temp': 'bot_lm20_temp',
            '5v': 'bot_pwr_5v',
            '2.5v': 'bot_pwr_2_5v',
            '3.3v': 'bot_pwr_3_3v',
            '1.25v': 'bot_pwr_1_25v',
            '1.2v': 'bot_pwr_1_2v',
            'V in': 'bot_pwr_v_in'
        }

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

        # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

    @Slot(bool)
    def on_pushButton_record_on_clicked(self, checked):
        rospy.loginfo("On a presse sur le bouton On du record!")
        pass

    @Slot(bool)
    def on_pushButton_record_off_clicked(self, checked):
        rospy.loginfo("On a presse sur le bouton Off du record!")
        pass

    def cb_status(self, msg):
        rospy.loginfo("cb_status - msg.laser_state: {}".format(msg.laser_state))
        self._VLP16Status = msg

    def cb_diagnostics(self, msg):
        rospy.loginfo("cb_diagnostics - msg.top_ad_temp: {}".format(msg.top_ad_temp))
        self._VLP16Diagnostics = msg

    def on_update_GUI_status_timer(self):
        if self._VLP16Status:
            self._widget.textEdit_laser_state.setText(str(self._VLP16Status.laser_state))
            self._widget.textEdit_motor_state.setText(str(self._VLP16Status.motor_state))
            self._widget.textEdit_motor_rpm.setText(str(self._VLP16Status.motor_rpm))
            self._widget.textEdit_pps_state.setText(str(self._VLP16Status.gps_state))
            self._widget.textEdit_gps_position.setText(str(self._VLP16Status.gps_position))
        #        rospy.loginfo("on_laser_state - self._laser_state: {}".format(self._laser_state))

    @staticmethod
    def _update_gui_from_ros_msg(qt_tree, ros_msg, map_tree_to_ros_msg):
        # url: http://stackoverflow.com/questions/8961449/pyqt-qtreewidget-iterating
        for row in range(qt_tree.childCount()):
            child = qt_tree.child(row)
            field = child.text(0)
            #
#                rospy.loginfo("dir(child): {}".format(dir(child)))
            # rospy.loginfo("field : {}".format(field))
            try:
                #
                ros_field_in_msg = map_tree_to_ros_msg[field]
                # rospy.loginfo("fros_field_in_msg : {}".format(ros_field_in_msg))
                # url:
                # http://stackoverflow.com/questions/2612610/how-to-access-object-attribute-given-string-corresponding-to-name-of-that-attrib
                value_in_ros_msg = getattr(ros_msg, ros_field_in_msg)
                # rospy.loginfo("value_in_ros_msg : {}".format(value_in_ros_msg))
                #
                child.setText(1, str(value_in_ros_msg))
            except KeyError:
                pass

    def on_update_GUI_diagnostics_timer(self):
        if self._VLP16Diagnostics:
            tree = self._widget.treeWidget_diag
            tree_top_board = tree.topLevelItem(0)
            tree_bot_board = tree.topLevelItem(1)
            #
            # rospy.loginfo("tree_top_board.childCount(): {}".format(str(tree_top_board.childCount())))
            # rospy.loginfo("tree_bot_board.childCount(): {}".format(str(tree_bot_board.childCount())))
            #
            self._update_gui_from_ros_msg(tree_top_board, self._VLP16Diagnostics, self.map_top_qt_ros_msg)
            self._update_gui_from_ros_msg(tree_bot_board, self._VLP16Diagnostics, self.map_bot_qt_ros_msg)
