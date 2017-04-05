from li3ds_tab_record import LI3DSPlugin_Record
import time
import sys
from subprocess import Popen
from li3ds_tools import terminate_process_and_children, count_children_of_process
import rospy

import PyQt4.QtGui as QtGui
import PyQt4.QtCore as QtCore


class LI3DSPlugin_Record_RosBag(LI3DSPlugin_Record):

    # urls:
    # - http://stackoverflow.com/questions/14410152/pyqt-on-click-open-new-window
    # - http://stackoverflow.com/questions/19442443/busy-indication-with-pyqt-progress-bar
    class DialogForSequenceRecord(QtGui.QDialog):
        # taskStart = QtCore.pyqtSignal()
        # taskFinished = QtCore.pyqtSignal()

        def __init__(self, parent=None, li3ds_plugin_record=None):
            """

            :param parent:
            :param li3ds_plugin_record:
            """
            super(LI3DSPlugin_Record_RosBag.DialogForSequenceRecord, self).__init__(parent)

            self._li3ds_plugin_record = li3ds_plugin_record

            self.buttonBox = QtGui.QDialogButtonBox(self)
            self.buttonBox.setOrientation(QtCore.Qt.Horizontal)

            self.textBrowser = QtGui.QTextBrowser(self)

            self.verticalLayout = QtGui.QVBoxLayout(self)
            self.verticalLayout.addWidget(self.textBrowser)
            self.verticalLayout.addWidget(self.buttonBox)

            # Create a progress bar and a button and add them to the main layout
            self.progressBar = QtGui.QProgressBar(self)
            self.progressBar.setRange(0, 100)
            self.verticalLayout.addWidget(self.progressBar)

            self.taskRecordSequence = LI3DSPlugin_Record_RosBag.TaskThread(
                parent=self,
                li3ds_plugin_record=self._li3ds_plugin_record,
                textBrowser=self.textBrowser
            )
            self.taskRecordSequence.notifyProgress.connect(self.onProgress)
            self.taskRecordSequence.taskFinished.connect(self.onFinished)

            self._sequence_for_synch = True

        @property
        def sequence_for_synch(self):
            return self._sequence_for_synch

        @sequence_for_synch.setter
        def sequence_for_synch(self, value):
            """

            :return:
            """
            self._sequence_for_synch = value

        def showEvent(self, event):
            self.textBrowser.append("showEvent")
            #
            self.onStart()

        def onStart(self):
            self.textBrowser.append("onStart")
            #
            self.progressBar.setRange(0, 100)
            #
            self.taskRecordSequence.sequence_for_synch = self._sequence_for_synch
            self.taskRecordSequence.start()

        def onProgress(self, i):
            self.progressBar.setValue(i)

        def onFinished(self):
            self.textBrowser.append("onFinished")
            # Stop
            self.progressBar.setValue(100)
            #
            self.textBrowser.clear()
            self.textBrowser.clearHistory()
            #
            self.close()

    class TaskThread(QtCore.QThread):
        taskFinished = QtCore.pyqtSignal()
        notifyProgress = QtCore.pyqtSignal(int)

        def __init__(self,
                     parent=None,
                     li3ds_plugin_record=None,
                     textBrowser=None,
                     sequence_for_synch=True
                     ):
            """

            :param parent:
            :param li3ds_plugin_record:
            :param textBrowser:
            :param sequence_for_synch:
            """
            super(LI3DSPlugin_Record_RosBag.TaskThread, self).__init__(parent)

            self._li3ds_plugin_record = li3ds_plugin_record
            self._textBroswer = textBrowser
            self._sequence_for_synch = sequence_for_synch

        @property
        def sequence_for_synch(self):
            return self._sequence_for_synch

        @sequence_for_synch.setter
        def sequence_for_synch(self, value):
            """

            :return:
            """
            self._sequence_for_synch = value

        def run(self):
            # launch record sequence
            msg = "WAIT => on place la camera en position 'start' ..."
            self._textBroswer.append(msg)
            self._li3ds_plugin_record._launch_sequence_start_record_state_set_cam_on_start()
            self.notifyProgress.emit(10)

            # [RECORD - BAG]
            msg = "RosBag Record: START"
            self._textBroswer.append(msg)
            self._li3ds_plugin_record._launch_sequence_start_record_state_launch_rosbag_record()
            self.notifyProgress.emit(50)

            if self._sequence_for_synch:
                # On place la camera en position 'stop' (via une commande Arduino)
                msg = "WAIT => on place la camera en position 'stop' ..."
                self._textBroswer.append(msg)
                self._li3ds_plugin_record._launch_sequence_start_record_state_set_cam_on_stop()
                self.notifyProgress.emit(60)

                # -> WAIT 3s => Temporisation cote CamLight comme repere temporel
                msg = "WAIT 3s => Temporisation cote CamLight comme repere temporel"
                self._textBroswer.append(msg)
                self._li3ds_plugin_record._launch_sequence_start_record_state_wait_for_temporal_synch()
                self.notifyProgress.emit(90)

                # On replace la camera en position 'start' (via une commande Arduino)
                msg = "WAIT => on place la camera en position 'start' ..."
                self._textBroswer.append(msg)
                self._li3ds_plugin_record._launch_sequence_start_record_state_set_cam_on_start()
                self.notifyProgress.emit(100)

            #
            self.taskFinished.emit()

    def __init__(self, *args, **kwargs):
        """

        :param args:
        :param kwargs:
        """
        super(LI3DSPlugin_Record_RosBag, self).__init__(*args, **kwargs)

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
        self.gui.widget.listWidget_record_settings.addItems(self._rosbag_topic_names)

        self.dialog_sequence_record = LI3DSPlugin_Record_RosBag.DialogForSequenceRecord(
            self.gui.widget,
            li3ds_plugin_record=self
        )

        self._rosbag_process = None

    def _launch_sequence_start_record_state_set_cam_on_start(self):
        # On place la camera en position 'start' (via une commande Arduino)
        self.states.set_state('arduino', 'start', state=True,
                              update_label_pixmap=self.gui.update_label_pixmap_onoff, update_ros=True)
        rospy.loginfo("WAIT 1s => On place la camera en position 'start' ...")
        # -> WAIT 1s => transmission du message vers l'arduino
        # time.sleep(1)
        while self._li3ds_plugin._tab_arduino._msg_arduino_states.state_start:
            time.sleep(0.1)

    def _launch_sequence_start_record_state_launch_rosbag_record(self):
        # [RECORD - BAG]
        rospy.loginfo("RosBag Record: START")
        rospy.loginfo("WAIT => Lancement du rosbag record ...")
        self._rosbag_start_record()

    def _launch_sequence_start_record_state_set_cam_on_stop(self):
        # On place la camera en position 'start' (via une commande Arduino)
        self.states.set_state('arduino', 'start', state=False,
                              update_label_pixmap=self.gui.update_label_pixmap_onoff, update_ros=True)
        rospy.loginfo("WAIT 1s => On place la camera en position 'stop' ...")
        # -> WAIT 1s => transmission du message vers l'arduino
        # time.sleep(1)
        while not self._li3ds_plugin._tab_arduino._msg_arduino_states.state_start:
            time.sleep(0.1)

    def _launch_sequence_start_record_state_wait_for_temporal_synch(self, wait_nb_sec=3):
        """

        :param wait_nb_sec:
        :return:
        """
        rospy.loginfo("WAIT %ds => Temporisation cote CamLight comme repere temporel" % wait_nb_sec)
        time.sleep(wait_nb_sec)

    def _launch_sequence_start_record(self,
                                      sequence_for_synch=True,
                                      use_dialog=True):
        """

        :param sequence_for_synch:
        :param use_dialog:
        :return:
        """
        # TODO: faudrait une machine a etat pour gerer la sequence de start avec temporisation sans etre blocante ...

        # ELLAPSED-TIME
        self._rosbag_start_time = time.time()

        if use_dialog:
            self.dialog_sequence_record.sequence_for_synch = sequence_for_synch
            self.dialog_sequence_record.exec_()
        else:
            # launch record sequence
            # Cam sur 'start'
            self._launch_sequence_start_record_state_set_cam_on_start()
            # Lancement de RosBag record
            self._launch_sequence_start_record_state_launch_rosbag_record()
            if sequence_for_synch:
                # On place la camera en position 'stop' (via une commande Arduino)
                self._launch_sequence_start_record_state_set_cam_on_stop()
                # -> WAIT 3s => Temporisation cote CamLight comme repere temporel
                self._launch_sequence_start_record_state_wait_for_temporal_synch()
                # On replace la camera en position 'start' (via une commande Arduino)
                self._launch_sequence_start_record_state_set_cam_on_start()

    def _launch_sequence_stop_record(self, *args, **kwargs):
        """

        :param args:
        :param kwargs:
        :return:
        """
        # On place la camera en position 'stop' (via une commande Arduino)
        self.states.set_state('arduino', 'start', state=False,
                              update_label_pixmap=self.gui.update_label_pixmap_onoff, update_ros=True)
        rospy.loginfo("WAIT 1s => On place la camera en position 'stop' ...")
        # -> Wait 1s => transmission du message vers l'arduino
        time.sleep(1)

        # [RECORD - BAG]
        rospy.loginfo("RosBag Record: STOP")
        self._rosbag_stop_record(*args, **kwargs)

        self.loginfo('record', '[STOP] Ellapsed time from [START]: %s' % (time.time() - self._rosbag_start_time))

    def _rosbag_build_name(self):
        """

        :return:
        """
        self._rosbag_filename = '%s/session%s_section%s_%s' % (self._rosbag_path_to_record,
                                                               self._rosbag_session,
                                                               self._rosbag_section,
                                                               str(rospy.get_rostime()))

    def _rosbag_start_record(self, wait_for_child_process=True):
        """

        :param wait_for_child_process:
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

        if wait_for_child_process:
            while count_children_of_process(self._rosbag_process) != 1:
                time.sleep(0.1)

        # internal log
        self.loginfo('record', '[START] Record RosBag: %s' % self._rosbag_filename)

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
            self.loginfo('record', '[STOP] Record RosBag - pid: %s' % self._rosbag_process.pid)

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
        self.states.set_state('record', 'pause', state=True,
                              update_label_pixmap=self.gui.update_label_enable)
        #
        self._launch_sequence_stop_record(ending_session=False, reset_section=False)

        # ELLAPSED-TIME
        if self._rosbag_pause_time:
            self.loginfo('record', '[PAUSE] Ellapsed time from last [PAUSE]: %s'
                         % (time.time() - self._rosbag_pause_time))
        # self._add_to_log('record', '[PAUSE] Ellapsed time from last [START]: %s'
        #                  % (time.time() - self._rosbag_start_time))

        self._rosbag_pause_time = time.time()

    def _rosbag_unpause_record(self):
        """
        """
        #
        self.states.set_state('record', 'pause', state=False,
                              update_label_pixmap=self.gui.update_label_enable)
        #
        self._launch_sequence_start_record(sequence_for_synch=False)
