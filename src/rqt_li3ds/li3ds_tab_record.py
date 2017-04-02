from PyQt4.QtCore import pyqtSlot, QTimer
import rospy
from li3ds_tabs import ILI3DSPlugin_Tabs


class LI3DSPlugin_Record(ILI3DSPlugin_Tabs):
    def __init__(self, *args, **kwargs):
        """

        :param args:
        :param kwargs:
        """
        super(LI3DSPlugin_Record, self).__init__(*args, **kwargs)

        # tab: Record
        self.states.set_state('record', 'camlight')
        self.states.set_state('record', 'ins')
        self.states.set_state('record', 'vlp16')
        self.states.set_state('record', 'arduino')
        #
        self.states.set_state('record', 'pause', state=False)

        # ---------------------------------
        # SETTING des connections slots
        # ---------------------------------
        self.gui.widget.pushButton_record_on.clicked[bool].connect(self.on_pushButton_record_on_clicked)
        self.gui.widget.pushButton_record_off.clicked[bool].connect(self.on_pushButton_record_off_clicked)
        self.gui.widget.pushButton_record_pause.clicked[bool].connect(self.on_pushButton_record_pause_clicked)

        self._update_record_pixmaps()

        # Record - Status
        self._update_gui_record_status_timer = QTimer()
        self._update_gui_record_status_timer.timeout.connect(self._on_update_gui_record_status_timer)
        self._update_gui_record_status_timer.setInterval(500)

        self._update_gui_record_status_timer.start()

        self._devices_states_for_record = {}

    def _update_record_pixmaps(self):
        """
        """
        self.gui.update_label_pixmap_onoff('record')
        #
        self.gui.update_label_pixmap_onoff('record', 'arduino')
        self.gui.update_label_pixmap_onoff('record', 'ins')
        self.gui.update_label_pixmap_onoff('record', 'vlp16')
        self.gui.update_label_pixmap_onoff('record', 'camlight')

    @pyqtSlot(bool)
    def on_pushButton_record_on_clicked(self, checked):
        rospy.loginfo("[Record] - 'ON' button pushed!")
        if not self.states.get_state('record'):
            #
            self.states.set_state('record', state=True, update_label_pixmap=self.gui.update_label_pixmap_onoff)
            #
            self._launch_sequence_start_record()

    @pyqtSlot(bool)
    def on_pushButton_record_off_clicked(self, checked):
        rospy.loginfo("[Record] - 'OFF' button pushed!")
        if self.states.get_state('record'):
            #
            self.states.set_state('record', state=False, update_label_pixmap=self.gui.update_label_pixmap_onoff)
            #
            self.states.set_state('record', 'pause', state=False, update_label_pixmap=self.gui.update_label_enable)
            #
            self._launch_sequence_stop_record()

    @pyqtSlot(bool)
    def on_pushButton_record_pause_clicked(self, checked):
        rospy.loginfo("[Record] - 'PAUSE' button pushed!")
        if self.states.get_state('record'):
            if self.states.get_state('record', 'pause'):
                self.states.set_state('record', 'pause', state=False, update_label_pixmap=self.gui.update_label_enable)
                #
                self._rosbag_unpause_record()
            else:
                self.states.set_state('record', 'pause', state=True, update_label_pixmap=self.gui.update_label_enable)
                #
                self._rosbag_pause_record()
    
    def _on_update_gui_record_status_timer(self):
        """

        :return:
        """
        # La camera est prete pour l'enregistrement si:
        # - elle est bootee
        # - et l'etat est valide
        # - si l'arduino est en phase de 'start' (trig pour les captures images)
        # - si l'arduino n'est pas en 'pause'
        self._devices_states_for_record['camlight'] = self.states.get_state('camlight', 'boot')
        self._devices_states_for_record['camlight'] &= (not self.states.get_state('camlight', 'validate'))
        # self._devices_states_for_record['camlight'] &= self._get_state('arduino', 'start')
        # self._devices_states_for_record['camlight'] &= (not self._get_state('arduino', 'pause'))
        # Arduino est pret pour l'enregistrement si connection ROS etablie
        # TODO: probleme de design a ce niveau. IL faut un devices/tabs manager !
        # Record ne devrait pas rentrer dans la structure de 'li3ds_plugin' pour chercher les etats des devices !
        _pub_arduino_commands = self._li3ds_plugin._tab_arduino._pub_arduino_commands
        if _pub_arduino_commands:
            self._devices_states_for_record['arduino'] = _pub_arduino_commands.get_num_connections() > 0
        else:
            self._devices_states_for_record['arduino'] = False
        # Laser (vlp16) est pret pour l'enregistrement si son statut est sur 'true' (statut recepure par message
        # ROS transmis par le node qui communique avec le webserver du VLP-16)
        # TODO: probleme de design a ce niveau. IL faut un devices/tabs manager !
        _msg_vlp16_status = self._li3ds_plugin._tab_vlp16._msg_vlp16_status
        if _msg_vlp16_status:
            self._devices_states_for_record['vlp16'] = _msg_vlp16_status.laser_state
        else:
            self._devices_states_for_record['vlp16'] = False
        #
        # TODO: probleme de design a ce niveau. IL faut un devices/tabs manager !
        _msg_ins_status = self._li3ds_plugin._tab_ins._msg_ins_status
        if _msg_ins_status:
            self._devices_states_for_record['ins'] = _msg_ins_status.generalStatus == 63
        else:
            self._devices_states_for_record['ins'] = False  # internal value of SBG lib
        #
        #
        self.states.set_state('record', 'camlight', self._devices_states_for_record['camlight'])
        self.states.set_state('record', 'arduino', self._devices_states_for_record['arduino'])
        self.states.set_state('record', 'vlp16', self._devices_states_for_record['vlp16'])
        self.states.set_state('record', 'ins', self._devices_states_for_record['ins'])
        #
        self._update_record_pixmaps()
