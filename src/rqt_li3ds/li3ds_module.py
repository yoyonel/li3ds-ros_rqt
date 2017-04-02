import os

import rospkg
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
# url: http://pyqt.sourceforge.net/Docs/PyQt4/new_style_signals_slots.html
from PyQt4.QtGui import QWidget

from li3ds_tab_arduino import LI3DSPlugin_Arduino
from li3ds_tab_vlp16 import LI3DSPlugin_VLP16
from li3ds_tab_ins import LI3DSPlugin_INS
from li3ds_tab_camlight import LI3DSPlugin_CamLight
from li3ds_states import LI3DSPlugin_States

# from li3ds_tab_record import LI3DSPlugin_Record
from li3ds_tab_record_rosbag import LI3DSPlugin_Record_RosBag
from li3ds_ros import LI3DSPlugin_ROS
from li3ds_gui import LI3DSPlugin_GUI


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
            self._widget.setWindowTitle(
                self._widget.windowTitle() +
                (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        #
        self._li3ds_states = LI3DSPlugin_States(self)
        self._li3ds_gui = LI3DSPlugin_GUI(self._widget, self._li3ds_states)
        self._li3ds_ros = LI3DSPlugin_ROS()
        #
        self._managers_for_tabs = {
            'li3ds_states': self._li3ds_states,
            'li3ds_gui': self._li3ds_gui,
            'li3ds_ros': self._li3ds_ros,
            'li3ds_plugin': self
        }
        self._tab_arduino = LI3DSPlugin_Arduino(**self._managers_for_tabs)
        self._tab_vlp16 = LI3DSPlugin_VLP16(**self._managers_for_tabs)
        self._tab_ins = LI3DSPlugin_INS(**self._managers_for_tabs)
        self._tab_camlight = LI3DSPlugin_CamLight(**self._managers_for_tabs)
        #
        self._tab_record = LI3DSPlugin_Record_RosBag(**self._managers_for_tabs)

    @property
    def li3ds_ros(self):
        """

        :return:
        """
        return self._li3ds_ros

    def loginfo(self, device, msg, roslog=True):
        """

        :param device:
        :param msg:
        """
        # url: http://stackoverflow.com/questions/7771164/add-more-than-one-line-to-a-qtextedit-pyqt
        self._widget.__dict__.get('textEdit_%s_log' % device, None).append(msg)
        #
        if roslog:
            rospy.loginfo(msg)

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
