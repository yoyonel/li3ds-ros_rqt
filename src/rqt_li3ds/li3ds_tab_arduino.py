import os
import rospkg

from li3ds_tabs import ILI3DSPlugin_Tabs
from PyQt4.QtCore import pyqtSlot, QTimer, pyqtSignal
import rospy

from li3ds_tools import iternodes, AudioFile, AudioFileWithPygame


class LI3DSPlugin_Arduino(ILI3DSPlugin_Tabs):
    # Qt Signals
    signal_arduino_state_start_on = pyqtSignal()
    signal_arduino_state_start_off = pyqtSignal()

    def __init__(self, *args, **kwargs):
        """

        :param args:
        :param kwargs:
        """
        super(LI3DSPlugin_Arduino, self).__init__(*args, **kwargs)

        # tab: Arduino
        self.states.set_state('arduino', 'flash')
        self.states.set_state('arduino', 'start')
        self.states.set_state('arduino', 'pause')

        self.gui.widget.pushButton_arduino_start.clicked[bool].connect(self.on_pushButton_arduino_start_clicked)
        self.gui.widget.pushButton_arduino_flash.clicked[bool].connect(self.on_pushButton_arduino_flash_clicked)
        self.gui.widget.pushButton_arduino_pause.clicked[bool].connect(self.on_pushButton_arduino_pause_clicked)
        #
        self._update_arduino_states_pixmaps()

        # Arduino - States
        self._update_gui_arduino_states_timer = QTimer()
        self._update_gui_arduino_states_timer.timeout.connect(self._on_update_gui_arduino_states_timer)
        self._update_gui_arduino_states_timer.setInterval(500)
        # Arduino - Diagnostics
        self._update_gui_arduino_diags_timer = QTimer()
        self._update_gui_arduino_diags_timer.timeout.connect(self._on_update_gui_arduino_diagnostics_timer)
        self._update_gui_arduino_diags_timer.setInterval(500)

        self._update_gui_arduino_states_timer.start()
        self._update_gui_arduino_diags_timer.start()

        # url:
        # ps: le tableau t2_t3_t4[3] est converti par rosmsg comme un chr()*3 (par soucy d'optimisation j'imagine).
        # Du coup, on ne recupere pas directement la valeur de l'entier mais la correspondance char.
        self.map_arduino_to_qt_ros_msg = {
            '/GPS/Clock/t2': lambda msg: ord(msg.t2_t3_t4[0]),
            '/GPS/Clock/t3': lambda msg: ord(msg.t2_t3_t4[1]),
            '/GPS/Clock/t4': lambda msg: ord(msg.t2_t3_t4[2]),
            '/GPS/PPS': lambda msg: 0,
            '/CamLight/nb_trigs': lambda msg: msg.num_trigs_for_pics,
            '/CamLight/states/boot': lambda msg: msg.state_boot
        }
        # '/CamLight/time_between_pics': lambda msg: msg.time_between_pics,

        # ---------------------------------
        # PUBLISHERS
        # ---------------------------------
        self._pub_arduino_commands, self._msg_arduino_commands = self.ros.find_publisher('/Arduino/sub/cmds')
        # ---------------------------------

        # ---------------------------------
        # SUBSCRIBERS
        # ---------------------------------
        # Arduino States
        self._sub_arduino_states = self.ros.subscribe_to_topic(
            '/Arduino/pub/states',
            self._cb_arduino_states, queue_size=10
        )

        self._msg_arduino_states = None

        self._play_sound = True
        rp = rospkg.RosPack()
        self.path_to_sound = path_to_sound = os.path.join(rp.get_path('rqt_li3ds'), 'resource',
                                                          'Click2-Sebastian-759472264.wav')
        # urls:
        # - https://github.com/Katee/quietnet/issues/18
        # - http://stackoverflow.com/questions/17657103/how-to-play-wav-file-in-python
        # - http://soundbible.com/1705-Click2.html
        self._sounds = {
            # 'click': AudioFile(path_to_sound),
            'click_pygame': AudioFileWithPygame(path_to_sound)
        }
        self.li3ds_plugin.loginfo('arduino', 'path_to_sound: %s' % path_to_sound)

    def _update_arduino_states_pixmaps(self):
        """
        """
        self.gui.update_label_pixmap_onoff('arduino', 'flash')
        self.gui.update_label_pixmap_onoff('arduino', 'start')
        self.gui.update_label_pixmap_onoff('arduino', 'pause')

    @pyqtSlot(bool)
    def on_pushButton_arduino_start_clicked(self, checked):
        rospy.loginfo("[Arduino][Commands] - 'Start' button pushed!")
        #
        state = self.states.switch_state('arduino', 'start',
                                         update_label_pixmap=self.gui.update_label_pixmap_onoff, update_ros=True)
        #
        self.gui.widget.pushButton_arduino_start.setText(str(('Disable' if state else 'Enable') + ' Start'))

    @pyqtSlot(bool)
    def on_pushButton_arduino_flash_clicked(self, checked):
        rospy.loginfo("[Arduino][Commands] - 'Flash' button pushed!")
        state = self.states.switch_state('arduino', 'flash',
                                         update_label_pixmap=self.gui.update_label_pixmap_onoff, update_ros=True)
        self.gui.widget.pushButton_arduino_flash.setText(str(('Disable' if state else 'Enable') + ' Flash'))

    @pyqtSlot(bool)
    def on_pushButton_arduino_pause_clicked(self, checked):
        rospy.loginfo("[Arduino][Commands] - 'Pause' button pushed!")
        state = self.states.switch_state('arduino', 'pause',
                                         update_label_pixmap=self.gui.update_label_pixmap_onoff, update_ros=True)
        self.gui.widget.pushButton_arduino_pause.setText(str(('Disable' if state else 'Enable') + ' Pause'))

    def _on_update_gui_arduino_states_timer(self):
        """

        """
        # Synch
        _msg_arduino_states = self._msg_arduino_states
        #
        if _msg_arduino_states:
            self.gui.widget.textEdit_arduino_flash.setText(str(_msg_arduino_states.state_flash))
            self.gui.widget.textEdit_arduino_pause.setText(str(_msg_arduino_states.state_pause))
            self.gui.widget.textEdit_arduino_start.setText(str(_msg_arduino_states.state_start))

    def _on_update_gui_arduino_diagnostics_timer(self):
        """
        Synchronisation du QTreeWidget 'self.gui.widget.treeWidget_arduino_diag' avec les valeurs dans le message ROS
         'self._msg_arduino_states' contenant les etats de l'Arduino.
        """
        if self._msg_arduino_states:
            root_path = ''
            for qt_item, path_to_qt_item in iternodes(
                    self.gui.widget.treeWidget_arduino_diag,
                    lambda t: zip([t.topLevelItem(topLevel) for topLevel in range(t.topLevelItemCount())],
                                  [root_path] * t.topLevelItemCount()),
                    lambda t: [t.child(id_child) for id_child in range(t.childCount())]
            ):
                # print('%s' % path_to_node)
                lambda_get_value_from_msg = self.map_arduino_to_qt_ros_msg.get(path_to_qt_item, lambda msg: None)
                value_in_ros_msg = lambda_get_value_from_msg(self._msg_arduino_states)
                if value_in_ros_msg is not None:
                    qt_item.setText(1, str(value_in_ros_msg))

    def _cb_arduino_states(self, msg):
        """

        :param msg:
        :type msg: arduino_msgs.msg.states
        """
        # rospy.loginfo("_cb_arduino_states - type(msg): %s" % type(msg))
        self._msg_arduino_states = msg

        if msg.state_start:
            self.signal_arduino_state_start_on.emit()
        else:
            self.signal_arduino_state_start_off.emit()

        if self._play_sound & msg.state_start & (not msg.state_pause):
            # self._sounds['click'].play()
            self._sounds['click_pygame'].play()

    def connect_signal(self, signal, cb):
        """

        :param signal:
        :param cb:
        :return:
        """
        # self.__class__.__dict__.get(signal).connect(cb)
        # self.__class__.__dict__.get('signal_arduino_state_start_on').connect(cb)
        self.signal_arduino_state_start_on.connect(cb)

    def _publish_arduino_states(self):
        """

        """
        msg = self._msg_arduino_commands()
        #
        msg.update_clock = False
        msg.t2_t3_t4 = [0] * 3
        #
        msg.state_flash = self.states.get_state('arduino', 'flash')
        msg.state_start = self.states.get_state('arduino', 'start')
        msg.state_pause = self.states.get_state('arduino', 'pause')
        #
        msg.state_boot = self.states.get_state('camlight', 'boot')
        #
        self._pub_arduino_commands.publish(msg)

    def state_updated(self):
        """

        :return:

        """
        try:
            if self._pub_arduino_commands:
                self._publish_arduino_states()
        except AttributeError as e:
            rospy.logwarn('_state_updated()')
