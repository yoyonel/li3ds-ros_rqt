import os

import rospkg
import rospy
import rostopic

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget, QLabel
from python_qt_binding.QtCore import Slot
from PyQt4.QtCore import QTimer

import sys

# from std_msgs.msg import String
from velodyne_configuration.msg import VLP16_StatusMessage
from velodyne_configuration.msg import VLP16_DiagnosticsMessage
#
from arduino_msgs.msg import commands
from arduino_msgs.msg import states
#
from sbg_driver.msg import SbgLogStatusData

import rosbag
from subprocess import Popen, PIPE
import signal
import subprocess

import time


def terminate_process_and_children(p):
    """

    url: http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/

    :param p:
    """
    ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % p.pid, shell=True, stdout=subprocess.PIPE)
    ps_output = ps_command.stdout.read()
    retcode = ps_command.wait()
    assert retcode == 0, "ps command returned %d" % retcode
    for pid_str in ps_output.split("\n")[:-1]:
        os.kill(int(pid_str), signal.SIGINT)
    p.terminate()


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
        ui_file = os.path.join(rp.get_path('rqt_li3ds'),
                               'resource', 'LI3DSPlugin.ui')
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
        # self._list_ros_topics = rospy.get_published_topics()
        # url: http://docs.ros.org/diamondback/api/rostopic/html/rostopic-pysrc.html
        self._master = rostopic.rosgraph.masterapi.Master('/rostopic')
        self._list_ros_topics = rostopic._master_get_topic_types(self._master)

        # ---------------------------------
        # PUBLISHERS
        # ---------------------------------
        self._pub_arduino_commands, self._msg_arduino_commands = self._find_publisher('/Arduino/sub/cmds')
        # ---------------------------------

        # ---------------------------------
        # SUBSCRIBERS
        # ---------------------------------
        # Arduino States
        self._sub_arduino_states = self._subscribe_to_topic(
            '/Arduino/pub/states',
            self._cb_arduino_states, queue_size=10
        )

        # VLP16: Status
        self._sub_vlp16_status = self._subscribe_to_topic(
            '/Laser/velodyne_status_server/status_pub',
            self._cb_vlp16_status, queue_size=10
        )
        # VLP16: Diagnostics
        self._sub_vlp16_diagnostics = self._subscribe_to_topic(
            '/Laser/velodyne_diagnostics_server/diagnostics_pub',
            self._cb_vlp16_diagnostics, queue_size=10
        )

        # INS: Status
        self._sub_ins_status = self._subscribe_to_topic(
            '/INS/SbgLogStatusData',
            self._cb_ins_status, queue_size=10
        )
        # ---------------------------------

        # dictionnaire des etats (GUI, Devices, ...)
        self._states = {}
        self._devices_states_for_record = {}

        # tab: Record
        self._set_state('record', 'camlight')
        self._set_state('record', 'ins')
        self._set_state('record', 'vlp16')
        self._set_state('record', 'arduino')
        #
        self._set_state('record', 'pause', state=False)

        # ---------------------------------
        # SETTING des connections slots
        # ---------------------------------
        #        rospy.loginfo("Test log!")
        self._widget.pushButton_record_on.clicked[bool].connect(self.on_pushButton_record_on_clicked)
        self._widget.pushButton_record_off.clicked[bool].connect(self.on_pushButton_record_off_clicked)
        self._widget.pushButton_record_pause.clicked[bool].connect(self.on_pushButton_record_pause_clicked)

        self._update_record_pixmaps()

        ##########
        # ARDUINO
        ##########
        self._set_state('arduino', 'flash')
        self._set_state('arduino', 'start')
        self._set_state('arduino', 'pause')
        #
        self._widget.pushButton_arduino_start.clicked[bool].connect(self.on_pushButton_arduino_start_clicked)
        self._widget.pushButton_arduino_flash.clicked[bool].connect(self.on_pushButton_arduino_flash_clicked)
        self._widget.pushButton_arduino_pause.clicked[bool].connect(self.on_pushButton_arduino_pause_clicked)
        #
        self._update_arduino_states_pixmaps()

        ############
        # CAMLIGHT
        ############
        self._set_state('camlight', 'boot')
        self._set_state('camlight', 'validate', True)
        #
        self._widget.pushButton_camlight_boot.clicked[bool].connect(self.on_pushButton_camlight_boot_clicked)
        self._widget.pushButton_camlight_validate.clicked[bool].connect(self.on_pushButton_camlight_validate_clicked)

        # Settings
        # ---------------------------------
        self._msg_vlp16_status = None
        self._msg_vlp16_diagnostics = None
        #
        self._msg_arduino_states = None
        #
        self._msg_ins_status = None

        # ---------------------------------
        # GUI: QT
        # ---------------------------------
        # Status
        self._update_gui_status_timer = QTimer()
        self._update_gui_status_timer.timeout.connect(self._on_update_gui_vlp16_status_timer)
        self._update_gui_status_timer.setInterval(500)
        # Diagnostics
        self._update_gui_diagnostics_timer = QTimer()
        self._update_gui_diagnostics_timer.timeout.connect(self._on_update_gui_diagnostics_timer)
        self._update_gui_diagnostics_timer.setInterval(500)
        # Arduino - States
        self._update_gui_arduino_states_timer = QTimer()
        self._update_gui_arduino_states_timer.timeout.connect(self._on_update_gui_arduino_states_timer)
        self._update_gui_arduino_states_timer.setInterval(500)
        # Arduino - Diagnostics
        self._update_gui_arduino_diags_timer = QTimer()
        self._update_gui_arduino_diags_timer.timeout.connect(self._on_update_gui_arduino_diagnostics_timer)
        self._update_gui_arduino_diags_timer.setInterval(500)
        # INS - Status
        self._update_gui_record_status_timer = QTimer()
        self._update_gui_record_status_timer.timeout.connect(self._on_update_gui_record_status_timer)
        self._update_gui_record_status_timer.setInterval(500)

        # Start QTimers
        self._update_gui_status_timer.start()
        self._update_gui_diagnostics_timer.start()
        self._update_gui_arduino_states_timer.start()
        self._update_gui_arduino_diags_timer.start()
        self._update_gui_record_status_timer.start()

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

        # url:
        # ps: le tableau t2_t3_t4[3] est converti par rosmsg comme un chr()*3 (pas soucy d'optimisation j'imagine).
        # Du coup, on ne recupere pas directement la valeur de l'entier mais ca correspondance char.
        self.map_arduino_to_qt_ros_msg = {
            '/GPS/Clock/t2': lambda msg: ord(msg.t2_t3_t4[0]),
            '/GPS/Clock/t3': lambda msg: ord(msg.t2_t3_t4[1]),
            '/GPS/Clock/t4': lambda msg: ord(msg.t2_t3_t4[2]),
            '/GPS/PPS': lambda msg: 0,
            '/CamLight/nb_trigs': lambda msg: msg.num_trigs_for_pics,
            '/CamLight/states/boot': lambda msg: msg.state_boot
        }

        # ROSBAG
        self._rosbag_split_duration = '1m'
        self._rosbag_path_to_record = '/root/project/records'
        self._rosbag_filename = ""
        #
        self._rosbag_session = 0
        self._rosbag_section = 0
        #
        self._rosbag_start_time = None
        self._rosbag_pause_time = None
        #
        self._rosbag_topic_names = [
            ##############################
            # Arduino
            ##############################
            '/Arduino/pub/states',
            # '/Arduino/sub/cmds',
            ##############################
            # INS: SBG Ellipse-N
            ##############################
            '/INS/SbgLogEkfNavData',
            '/INS/SbgLogEkfQuatData',
            '/INS/SbgLogGpsPos',
            '/INS/SbgLogGpsVel',
            '/INS/SbgLogImuData',
            '/INS/SbgLogMag',
            '/INS/SbgLogPressureData',
            '/INS/SbgLogShipMotionData',
            '/INS/SbgLogStatusData',
            '/INS/SbgLogUtcData',
            ##############################
            # Laser: VLP-16
            ##############################
            '/Laser/gpsimu_driver/gpstime',
            '/Laser/gpsimu_driver/imu_data',
            '/Laser/gpsimu_driver/nmea_sentence',
            '/Laser/gpsimu_driver/temperature',
            #
            # '/Laser/cloud_nodelet/parameter_descriptions',
            # '/Laser/cloud_nodelet/parameter_updates',
            # '/Laser/velodyne_nodelet_manager/bond',
            #  Nuages de points et packets reseaux
            # '/Laser/velodyne_packets',
            '/Laser/velodyne_points',
            #
            '/Laser/velodyne_diagnostics_server/diagnostics_pub',
            # '/Laser/velodyne_settings_server/parameter_descriptions',
            # '/Laser/velodyne_settings_server/parameter_updates',
            '/Laser/velodyne_settings_server/settings_pub',
            '/Laser/velodyne_status_server/status_pub',
            ##############################
            #
            # '/diagnostics',
            # '/rosout',
            # '/rosout_agg',
        ]

        # GUI
        # url: https://wiki.python.org/moin/PyQt/Adding%20items%20to%20a%20list%20widget
        # TODO: Effectuer une synchronisation entre la liste des topics enregistres et la liste des items
        # dans la listWidget Qt.
        self._widget.listWidget_record_settings.addItems(self._rosbag_topic_names)

    @staticmethod
    def _state_id(device, command=''):
        """

        :param device:
        :param command:
        :return:
        :rtype: str
        """
        return '%s%s%s' % (device, '_' if command else '', command)

    def _state_updated(self, device, command, update_label_pixmap=None, update_ros=False):
        """

        :param device:
        :param command:
        :param update_label_pixmap:
        :param update_ros:
        """
        # update GUI
        if update_label_pixmap:
            # self._update_label_pixmap_onoff(device, command)
            update_label_pixmap(device, command)
        # ROS
        if update_ros:
            try:
                if self._pub_arduino_commands:
                    self._publish_arduino_states()
            except AttributeError as e:
                rospy.logwarn('_state_updated(%s, %s' % (device, command))

    def _publish_arduino_states(self):
        """

        """
        msg = self._msg_arduino_commands()
        #
        msg.update_clock = False
        msg.t2_t3_t4 = [0] * 3
        #
        msg.state_flash = self._get_state('arduino', 'flash')
        msg.state_start = self._get_state('arduino', 'start')
        msg.state_pause = self._get_state('arduino', 'pause')
        #
        msg.state_boot = self._get_state('camlight', 'boot')
        #
        self._pub_arduino_commands.publish(msg)

    def _get_state(self, device, command=''):
        """

        :param device:
        :param command:
        :return:
        """
        return self._states.get(self._state_id(device, command), False)

    def _set_state(self, device, command='', state=False,
                   update_label_pixmap=None, update_ros=False):
        """

        :param device:
        :type device: str
        :param command:
        :type command: str
        :param state:
        :type state: boolean
        :param update_label_pixmap: [default=None]
        :type update_label_pixmap:
        :param update_ros: [default=False]
        :type update_ros: boolean
        """
        #
        self._states[self._state_id(device, command)] = state
        #
        self._state_updated(device, command, update_label_pixmap, update_ros)

    def _switch_state(self, device, command='',
                      update_label_pixmap=None, update_ros=False):
        """

        :param device:
        :param command:
        :param update_label_pixmap: [default=None]
        :type update_label_pixmap: 
        :param update_ros: [default=False]
        :type update_ros: boolean
        :return: new state
        :rtype: boolean
        """
        # Switch state
        # url:
        self._states[self._state_id(device, command)] ^= True
        self._state_updated(device, command, update_label_pixmap, update_ros)
        return self._states[self._state_id(device, command)]

    def _build_base_label_id(self, device, command=''):
        """

        :param device:
        :param command:
        :return:
        """
        #
        state_id = self._state_id(device, command)
        #
        state = self._states.get(state_id, False)
        #
        id_label = 'label_%s' % state_id

        return id_label, state

    def _update_label_pixmap_onoff(self, device, command=''):
        """

        :param device: ex: 'arduino'
        :type device: str
        :param command: ex: 'flash'
        :type command: str
        """
        label_id, state = self._build_base_label_id(device, command)
        label_id += '_pixmap'
        #
        label_id_on = '%s_%s' % (label_id, 'on')
        label_id_off = '%s_%s' % (label_id, 'off')
        # rospy.loginfo("id_label_for_pixmap: %s - %s" % (label_id_on, label_id_off))
        try:
            self._widget.__dict__.get(label_id_on, None).setEnabled(state)
            self._widget.__dict__.get(label_id_off, None).setEnabled(not state)
        except AttributeError as e:
            rospy.logerr('_update_label_pixmap_onoff(%s, %s)' % (device, command))
            rospy.logerr('Unexpected error: %s - error: %s',
                         sys.exc_info()[0], e)
        finally:
            return label_id, state

    def _update_label_enable(self, device, command=''):
        """

        :param device: ex: 'arduino'
        :type device: str
        :param command: ex: 'flash'
        :type command: str
        """
        #
        id_label, state = self._build_base_label_id(device, command)
        id_label += '_pixmap'
        #
        # rospy.loginfo("id label for pixmap: %s" % id_label)
        try:
            self._widget.__dict__.get(id_label, None).setEnabled(state)
        except AttributeError as e:
            rospy.logerr('_update_label_enable(%s, %s)' % (device, command))
            rospy.logerr('Unexpected error: %s - error: %s', sys.exc_info()[0], e)

    def _update_label_pixmap_camlight(self, device, command=''):
        """

        :param device:
        :param command:
        :return:
        """
        id_label, state = self._update_label_pixmap_onoff(device, command)
        self._widget.pushButton_camlight_boot.setText(str('HALT' if state else 'BOOT'))

    def _update_arduino_states_pixmaps(self):
        """
        """
        self._update_label_pixmap_onoff('arduino', 'flash')
        self._update_label_pixmap_onoff('arduino', 'start')
        self._update_label_pixmap_onoff('arduino', 'pause')

    def _update_record_pixmaps(self):
        """
        """
        self._update_label_pixmap_onoff('record')
        #
        self._update_label_pixmap_onoff('record', 'arduino')
        self._update_label_pixmap_onoff('record', 'ins')
        self._update_label_pixmap_onoff('record', 'vlp16')
        self._update_label_pixmap_onoff('record', 'camlight')

    def _find_publisher(
            self,
            pub_name_searched,
            queue_size=10,
            *args):
        """

        :param pub_name_searched:
        :param queue_size:
        :param args:
        :return:
        :rtype: (rospy.Publisher, genpy.Message)
        """
        ros_pub = None
        msg_name = 'None'
        try:
            pub_name, msg_name = self._search_ros_topic_message(pub_name_searched)
            rospy.logdebug('Publish to: %s with message type: %s', pub_name, msg_name)
            ros_pub = rospy.Publisher(pub_name, eval(msg_name), queue_size=queue_size, *args)
        except Exception as e:
            rospy.logwarn('_publish_to_topic(pub_name_searched=%s, *args=%s)', pub_name_searched, str(args))
            rospy.logwarn('Unexpected error: %s - error: %s',
                          sys.exc_info()[0], e)
        finally:
            return ros_pub, eval(msg_name)

    def _subscribe_to_topic(
            self,
            pub_name_searched,
            callback,
            queue_size=10):
        """
        """
        ros_sub = None
        try:
            pub_name, pub_msg = self._search_ros_topic_message(pub_name_searched)
            rospy.logdebug(
                'Subscribe to publisher: %s with message type: %s',
                pub_name, pub_msg)
            ros_sub = rospy.Subscriber(pub_name,
                                       eval(pub_msg),
                                       callback,
                                       queue_size=queue_size
                                       )
        except Exception as e:
            rospy.logerr('Unexpected error: %s - error: %s',
                         sys.exc_info()[0], e)
        finally:
            return ros_sub

    def _search_ros_topic_message(self, pub_name_searched, from_import=True):
        """
        """
        pub_name = None
        pub_msg = None
        try:
            pub_filtered = filter(lambda topic_msg: pub_name_searched in topic_msg[0], self._list_ros_topics)[0]
            pub_name, pub_msg = pub_filtered
            if from_import:
                pub_msg = pub_msg.split('/')[-1]
        except:
            rospy.logerr('Unexpected error: %s', sys.exc_info()[0])
        finally:
            return pub_name, pub_msg

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

    def _launch_sequence_start_record(self, sequence_for_synch=True):
        """

        TODO: pour l'instant la sequence est blocante. A revoir quand une FSM sera mise en place.
        """
        # ELLAPSED-TIME
        self._rosbag_start_time = time.time()

        # On place la camera en position 'start' (via une commande Arduino)
        self._set_state('arduino', 'start', state=True,
                        update_label_pixmap=self._update_label_pixmap_onoff, update_ros=True)
        rospy.loginfo("WAIT 1s => On place la camera en position 'start' ...")
        # -> WAIT 1s => transmission du message vers l'arduino
        time.sleep(1)

        # [RECORD - BAG]
        rospy.loginfo("RosBag Record: START")
        self._rosbag_start_record()
        rospy.loginfo("WAIT 5s => temporisation pour le lancement du rosbag record ...")
        # -> WAIT 5s => lancement du rosbag record
        time.sleep(5)

        if sequence_for_synch:
            # On place la camera en position 'stop' (via une commande Arduino)
            self._set_state('arduino', 'start', state=False,
                            update_label_pixmap=self._update_label_pixmap_onoff, update_ros=True)
            rospy.loginfo("WAIT 1s => On place la camera en position 'stop' ...")
            # -> Wait 1s => transmission du message vers l'arduino
            time.sleep(1)

            # -> WAIT 3s => Temporisation cote CamLight comme repere temporel
            rospy.loginfo("WAIT 3s => Temporisation cote CamLight comme repere temporel")
            time.sleep(3)

            # On replace la camera en position 'start' (via une commande Arduino)
            self._set_state('arduino', 'start', state=True,
                            update_label_pixmap=self._update_label_pixmap_onoff, update_ros=True)
            rospy.loginfo("WAIT 1s => On place la camera en position 'start' ...")
            # -> Wait 1s => transmission du message vers l'arduino
            time.sleep(1)

    def _launch_sequence_stop_record(self, *args, **kwargs):
        """

        :param args:
        :param kwargs:
        :return:
        """
        # On place la camera en position 'stop' (via une commande Arduino)
        self._set_state('arduino', 'start', state=False,
                        update_label_pixmap=self._update_label_pixmap_onoff, update_ros=True)
        rospy.loginfo("WAIT 1s => On place la camera en position 'stop' ...")
        # -> Wait 1s => transmission du message vers l'arduino
        time.sleep(1)

        # [RECORD - BAG]
        rospy.loginfo("RosBag Record: STOP")
        self._rosbag_stop_record(*args, **kwargs)

        self._add_to_log('record', '[STOP] Ellapsed time from [START]: %s' % (time.time() - self._rosbag_start_time))

    def _rosbag_build_name(self):
        """

        :return:
        """
        self._rosbag_filename = '%s/session%s_section%s_%s' % (self._rosbag_path_to_record,
                                                               self._rosbag_session,
                                                               self._rosbag_section,
                                                               str(rospy.get_rostime()))

    def _rosbag_start_record(self):
        """

        :return:
        """
        # url: http://wiki.ros.org/rosbag/Commandline#record
        # application: rosbag
        cmd = ['rosbag']
        # command: record
        cmd += ['record']
        # options
        # output name
        self._rosbag_build_name()
        cmd += ['-O', self._rosbag_filename]
        # split
        cmd += ['--split', '--duration=%s' % self._rosbag_split_duration]
        # compression
        cmd += ['--lz4']
        # topics to record
        cmd += self._rosbag_topic_names
        #
        self._rosbag_process = Popen(cmd)

        # internal log
        self._add_to_log('record', '[Start] Record RosBag: %s' % self._rosbag_filename)

    def _rosbag_stop_record(self,
                            ending_session=True,
                            reset_section=True):
        """

        :param ending_session:
        :type ending_session: bool
        :param reset_section:
        :type reset_section: bool
        :return:
        """
        if self._rosbag_process:
            try:
                terminate_process_and_children(self._rosbag_process)
            except AssertionError as e:
                rospy.logerr('Probleme pour terminer rosbag record ...')
                rospy.logerr('Unexpected error: %s - error: %s', sys.exc_info()[0], e)
            # internal log
            self._add_to_log('record', '[STOP] Record RosBag - pid: %s' % self._rosbag_process.pid)

            # Update Session/Section
            if ending_session:
                self._rosbag_session += 1
            #
            if reset_section:
                self._rosbag_section = 0
                # ELLAPSED-TIME
                self._rosbag_pause_time = None
            else:
                self._rosbag_section += 1

    def _rosbag_pause_record(self):
        """
        """
        #
        self._set_state('record', 'pause', state=True,
                        update_label_pixmap=self._update_label_enable)
        #
        self._launch_sequence_stop_record(ending_session=False, reset_section=False)

        # ELLAPSED-TIME
        if self._rosbag_pause_time:
            self._add_to_log('record', '[PAUSE] Ellapsed time from last [PAUSE]: %s'
                             % (time.time() - self._rosbag_pause_time))
        # self._add_to_log('record', '[PAUSE] Ellapsed time from last [START]: %s'
        #                  % (time.time() - self._rosbag_start_time))

        self._rosbag_pause_time = time.time()

    def _rosbag_unpause_record(self):
        """
        """
        #
        self._set_state('record', 'pause', state=False,
                        update_label_pixmap=self._update_label_enable)
        #
        self._launch_sequence_start_record(sequence_for_synch=False)

    @Slot(bool)
    def on_pushButton_record_on_clicked(self, checked):
        rospy.loginfo("[Record] - 'ON' button pushed!")
        if not self._get_state('record'):
            #
            self._set_state('record', state=True, update_label_pixmap=self._update_label_pixmap_onoff)
            #
            self._launch_sequence_start_record()


    @Slot(bool)
    def on_pushButton_record_off_clicked(self, checked):
        rospy.loginfo("[Record] - 'OFF' button pushed!")
        if self._get_state('record'):
            #
            self._set_state('record', state=False, update_label_pixmap=self._update_label_pixmap_onoff)
            #
            self._set_state('record', 'pause', state=False, update_label_pixmap=self._update_label_enable)
            #
            self._launch_sequence_stop_record()

    @Slot(bool)
    def on_pushButton_record_pause_clicked(self, checked):
        rospy.loginfo("[Record] - 'PAUSE' button pushed!")
        if self._get_state('record'):
            if self._get_state('record', 'pause'):
                self._set_state('record', 'pause', state=False,
                                update_label_pixmap=self._update_label_enable)
                #
                self._rosbag_unpause_record()
            else:
                self._set_state('record', 'pause', state=True,
                                update_label_pixmap=self._update_label_enable)
                #
                self._rosbag_pause_record()

    @Slot(bool)
    def on_pushButton_arduino_start_clicked(self, checked):
        rospy.loginfo("[Arduino][Commands] - 'Start' button pushed!")
        #
        state = self._switch_state('arduino', 'start',
                                   update_label_pixmap=self._update_label_pixmap_onoff, update_ros=True)
        #
        self._widget.pushButton_arduino_start.setText(str(('Disable' if state else 'Enable') + ' Start'))

    @Slot(bool)
    def on_pushButton_arduino_flash_clicked(self, checked):
        rospy.loginfo("[Arduino][Commands] - 'Flash' button pushed!")
        state = self._switch_state('arduino', 'flash',
                                   update_label_pixmap=self._update_label_pixmap_onoff, update_ros=True)
        self._widget.pushButton_arduino_flash.setText(str(('Disable' if state else 'Enable') + ' Flash'))

    @Slot(bool)
    def on_pushButton_arduino_pause_clicked(self, checked):
        rospy.loginfo("[Arduino][Commands] - 'Pause' button pushed!")
        state = self._switch_state('arduino', 'pause',
                                   update_label_pixmap=self._update_label_pixmap_onoff, update_ros=True)
        self._widget.pushButton_arduino_pause.setText(str(('Disable' if state else 'Enable') + ' Pause'))

    @Slot(bool)
    def on_pushButton_camlight_boot_clicked(self, checked):
        rospy.loginfo("[CamLight][Commands] - 'BOOT' button pushed!")
        # Ne met a jour que si 'camlight/validate' est en status: 'faux'
        if not self._get_state('camlight', 'validate'):
            self._switch_state('camlight', 'boot',
                               update_label_pixmap=self._update_label_pixmap_camlight, update_ros=True)
            #
            self._set_state('camlight', 'validate', state=True, update_label_pixmap=self._update_label_enable)

    @Slot(bool)
    def on_pushButton_camlight_validate_clicked(self, checked):
        rospy.loginfo("[CamLight][Commands] - 'BOOT' button pushed!")
        self._set_state('camlight', 'validate', state=False, update_label_pixmap=self._update_label_enable)

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

    def _cb_ins_status(self, msg):
        """

        :param msg:
        :type msg: SbgLogStatusData
        """
        # rospy.loginfo("_cb_ins_status- msg.generalStatus: {}".format(msg.generalStatus))
        self._msg_ins_status = msg

    def _cb_arduino_states(self, msg):
        """

        :param msg:
        :type msg: arduino_msgs.msg.states
        """
        # rospy.loginfo("_cb_arduino_states - type(msg): %s" % type(msg))
        self._msg_arduino_states = msg

    def _on_update_gui_vlp16_status_timer(self):
        """

        """
        if self._msg_vlp16_status:
            self._widget.textEdit_laser_state.setText(str(self._msg_vlp16_status.laser_state))
            self._widget.textEdit_motor_state.setText(str(self._msg_vlp16_status.motor_state))
            self._widget.textEdit_motor_rpm.setText(str(self._msg_vlp16_status.motor_rpm))
            self._widget.textEdit_pps_state.setText(str(self._msg_vlp16_status.gps_state))
            self._widget.textEdit_gps_position.setText(str(self._msg_vlp16_status.gps_position))

    def _on_update_gui_arduino_states_timer(self):
        """

        """
        if self._msg_arduino_states:
            self._widget.textEdit_arduino_flash.setText(str(self._msg_arduino_states.state_flash))
            self._widget.textEdit_arduino_pause.setText(str(self._msg_arduino_states.state_pause))
            self._widget.textEdit_arduino_start.setText(str(self._msg_arduino_states.state_start))

    @staticmethod
    def _update_gui_from_ros_msg(qt_tree, ros_msg, map_tree_to_ros_msg):
        """

        :param qt_tree:
        :param ros_msg:
        :param map_tree_to_ros_msg:
        """
        # url:
        # http://stackoverflow.com/questions/8961449/pyqt-qtreewidget-iterating
        for row in range(qt_tree.childCount()):
            child = qt_tree.child(row)
            field = child.text(0)
            try:
                ros_field_in_msg = map_tree_to_ros_msg[field]
                # url:
                # http://stackoverflow.com/questions/2612610/how-to-access-object-attribute-given-string-corresponding-to-name-of-that-attrib
                value_in_ros_msg = getattr(ros_msg, ros_field_in_msg)
                child.setText(1, str(value_in_ros_msg))
            except KeyError:
                pass

    def _on_update_gui_diagnostics_timer(self):
        """

        """
        if self._msg_vlp16_diagnostics:
            tree = self._widget.treeWidget_vlp16_diag
            #
            tree_top_board = tree.topLevelItem(0)
            tree_bot_board = tree.topLevelItem(1)
            #
            self._update_gui_from_ros_msg(tree_top_board, self._msg_vlp16_diagnostics, self.map_top_qt_ros_msg)
            self._update_gui_from_ros_msg(tree_bot_board, self._msg_vlp16_diagnostics, self.map_bot_qt_ros_msg)

    @staticmethod
    def _iternodes(tree, call_getroot, call_getchildren):
        """

        ps: on pourrait le rendre plus generique en stipulant des callers pour extraire les nodes, construire les paths,
        action(s) a effectuer sur les nodes, etc, ...

        url: http://stackoverflow.com/questions/8991840/recursion-using-yield

        :param tree: arbre (ex: QTreeWidget)
        :param call_getroot: caller pour recuperer les nodes roots (ou niveau 0)
        :param call_getchildren: caller pour recuperer les nodes enfants
        :return:
        """
        stack = call_getroot(tree)
        while stack:
            node, path_to_node = stack.pop()
            path_to_node += '/%s' % node.text(0)
            yield node, path_to_node
            for child in call_getchildren(node):
                stack.append((child, path_to_node))

    def _on_update_gui_arduino_diagnostics_timer(self):
        """
        Synchronisation du QTreeWidget 'self._widget.treeWidget_arduino_diag' avec les valeurs dans le message ROS
         'self._msg_arduino_states' contenant les etats de l'Arduino.
        """
        if self._msg_arduino_states:
            root_path = ''
            for qt_item, path_to_qt_item in self._iternodes(
                    self._widget.treeWidget_arduino_diag,
                    lambda t: zip([t.topLevelItem(topLevel) for topLevel in range(t.topLevelItemCount())],
                                  [root_path] * t.topLevelItemCount()),
                    lambda t: [t.child(id_child) for id_child in range(t.childCount())]
            ):
                # print('%s' % path_to_node)
                lambda_get_value_from_msg = self.map_arduino_to_qt_ros_msg.get(path_to_qt_item, lambda msg: None)
                value_in_ros_msg = lambda_get_value_from_msg(self._msg_arduino_states)
                if value_in_ros_msg is not None:
                    qt_item.setText(1, str(value_in_ros_msg))

    def _on_update_gui_record_status_timer(self):
        """

        :return:
        """
        # La camera est prete pour l'enregistrement si:
        # - elle est bootee
        # - et l'etat est valide
        # - si l'arduino est en phase de 'start' (trig pour les captures images)
        # - si l'arduino n'est pas en 'pause'
        self._devices_states_for_record['camlight'] = self._get_state('camlight', 'boot')
        self._devices_states_for_record['camlight'] &= (not self._get_state('camlight', 'validate'))
        # self._devices_states_for_record['camlight'] &= self._get_state('arduino', 'start')
        # self._devices_states_for_record['camlight'] &= (not self._get_state('arduino', 'pause'))
        # Arduino est pret pour l'enregistrement si connection ROS etablie
        self._devices_states_for_record['arduino'] = self._pub_arduino_commands.get_num_connections() > 0 if self._pub_arduino_commands else False
        # Laser (vlp16) est pret pour l'enregistrement si son statut est sur 'true' (statut recepure par message
        # ROS transmis par le node qui communique avec le webserver du VLP-16)
        self._devices_states_for_record['vlp16'] = self._msg_vlp16_status.laser_state if self._msg_vlp16_status else False
        #
        self._devices_states_for_record['ins'] = self._msg_ins_status.generalStatus == 63 if self._msg_ins_status else False  # internal value of SBG lib
        #

        #
        self._set_state('record', 'camlight', self._devices_states_for_record['camlight'])
        self._set_state('record', 'arduino', self._devices_states_for_record['arduino'])
        self._set_state('record', 'vlp16', self._devices_states_for_record['vlp16'])
        self._set_state('record', 'ins', self._devices_states_for_record['ins'])
        #
        self._update_record_pixmaps()

    def _add_to_log(self, device, msg):
        """

        :param device:
        :param msg:
        """
        # url: http://stackoverflow.com/questions/7771164/add-more-than-one-line-to-a-qtextedit-pyqt
        self._widget.__dict__.get('textEdit_%s_log' % device, None).append(msg)
