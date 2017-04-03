from PyQt4.QtCore import pyqtSlot, QTimer
import rospy

from li3ds_tabs import ILI3DSPlugin_Tabs

from li3ds_tools import update_gui_from_ros_msg


class LI3DSPlugin_VLP16(ILI3DSPlugin_Tabs):
    def __init__(self, *args, **kwargs):
        """

        :param args:
        :param kwargs:
        """
        super(LI3DSPlugin_VLP16, self).__init__(*args, **kwargs)

        # VLP16: Status
        self._sub_vlp16_status = self.ros.subscribe_to_topic(
            '/Laser/velodyne_status_server/status_pub',
            self._cb_vlp16_status, queue_size=10
        )
        # VLP16: Diagnostics
        self._sub_vlp16_diagnostics = self.ros.subscribe_to_topic(
            '/Laser/velodyne_diagnostics_server/diagnostics_pub',
            self._cb_vlp16_diagnostics, queue_size=10
        )

        # ---------------------------------
        # ROS: Messages
        # ---------------------------------
        self._msg_vlp16_status = None
        self._msg_vlp16_diagnostics = None

        # ---------------------------------
        # GUI: QT
        # ---------------------------------
        # VLP16 - Status
        self._update_gui_vlp16_status_timer = QTimer()
        self._update_gui_vlp16_status_timer.timeout.connect(self._on_update_gui_vlp16_status_timer)
        self._update_gui_vlp16_status_timer.setInterval(500)
        # VLP16 - Diagnostics
        self._update_gui_vlp16_diagnostics_timer = QTimer()
        self._update_gui_vlp16_diagnostics_timer.timeout.connect(self._on_update_gui_vlp16_diagnostics_timer)
        self._update_gui_vlp16_diagnostics_timer.setInterval(500)
        # Start QTimers
        self._update_gui_vlp16_status_timer.start()
        self._update_gui_vlp16_diagnostics_timer.start()

        self.map_vlp16_top_qt_ros_msg = {
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
        self.map_vlp16_bot_qt_ros_msg = {
            # Bottom Board
            'I out': 'bot_i_out',
            'Temp': 'bot_lm20_temp',
            '5v': 'bot_pwr_5v',
            '2.5v': 'bot_pwr_2_5v',
            '3.3v': 'bot_pwr_3_3v',
            '1.25v': 'bot_pwr_1_25v',
            '1.2v': 'bot_pwr_1_2v',
            'V in': 'bot_pwr_v_in'
        }

    @pyqtSlot(bool)
    def on_pushButton_camlight_validate_clicked(self, checked):
        rospy.loginfo("[CamLight][Commands] - 'BOOT' button pushed!")
        self.set_state('camlight', 'validate', state=False,
                       update_label_pixmap=self.gui.update_label_enable)

    def _cb_vlp16_status(self, msg):
        """

        :param msg:
        :type msg: VLP16_StatusMessage
        """
        # rospy.loginfo("_cb_vlp16_status - msg.laser_state: {}".format(msg.laser_state))
        self._msg_vlp16_status = msg

    def _cb_vlp16_diagnostics(self, msg):
        """

        :param msg:
        :type msg: VLP16_DiagnosticsMessage
        :return:
        """
        # rospy.loginfo("_cb_vlp16_diagnostics - msg.top_ad_temp: {}".format(msg.top_ad_temp))
        self._msg_vlp16_diagnostics = msg

    def _on_update_gui_vlp16_status_timer(self):
        """

        """
        if self._msg_vlp16_status:
            self.gui.widget.textEdit_laser_state.setText(str(self._msg_vlp16_status.laser_state))
            self.gui.widget.textEdit_motor_state.setText(str(self._msg_vlp16_status.motor_state))
            self.gui.widget.textEdit_motor_rpm.setText(str(self._msg_vlp16_status.motor_rpm))
            self.gui.widget.textEdit_pps_state.setText(str(self._msg_vlp16_status.gps_state))
            self.gui.widget.textEdit_gps_position.setText(str(self._msg_vlp16_status.gps_position))

    def _on_update_gui_vlp16_diagnostics_timer(self):
        """

        """
        if self._msg_vlp16_diagnostics:
            tree = self.gui.widget.treeWidget_vlp16_diag
            #
            tree_top_board = tree.topLevelItem(0)
            tree_bot_board = tree.topLevelItem(1)
            #
            update_gui_from_ros_msg(tree_top_board, self._msg_vlp16_diagnostics, self.map_vlp16_top_qt_ros_msg)
            update_gui_from_ros_msg(tree_bot_board, self._msg_vlp16_diagnostics, self.map_vlp16_bot_qt_ros_msg)
