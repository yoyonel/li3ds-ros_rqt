from PyQt4.QtCore import pyqtSlot, QTimer
import rospy

from li3ds_tabs import ILI3DSPlugin_Tabs

from li3ds_tools import update_gui_from_ros_msg


class LI3DSPlugin_CamLight(ILI3DSPlugin_Tabs):
    def __init__(self, *args, **kwargs):
        """

        :param args:
        :param kwargs:
        """
        super(LI3DSPlugin_CamLight, self).__init__(*args, **kwargs)

        ############
        # CAMLIGHT
        ############
        self.states.set_state('camlight', 'boot')
        self.states.set_state('camlight', 'validate', state=False)
        #
        self.gui.widget.pushButton_camlight_boot.clicked[bool].connect(self.on_pushButton_camlight_boot_clicked)
        self.gui.widget.pushButton_camlight_validate.clicked[bool].connect(self.on_pushButton_camlight_validate_clicked)

    def _update_label_pixmap_camlight(self, device, command=''):
        """

        :param device:
        :param command:
        :return:
        """
        id_label, state = self.gui.update_label_pixmap_onoff(device, command)
        self.gui.widget.pushButton_camlight_boot.setText(str('HALT' if state else 'BOOT'))

    @pyqtSlot(bool)
    def on_pushButton_camlight_boot_clicked(self, checked):
        rospy.loginfo("[CamLight][Commands] - 'BOOT' button pushed!")
        # Ne met a jour que si 'camlight/validate' est en status: 'faux'
        if not self.get_state('camlight', 'validate'):
            self.switch_state('camlight', 'boot',
                              update_label_pixmap=self._update_label_pixmap_camlight, update_ros=True)
            #
            self.states.set_state('camlight', 'validate', state=True, update_label_pixmap=self._update_label_enable)

    @pyqtSlot(bool)
    def on_pushButton_camlight_validate_clicked(self, checked):
        rospy.loginfo("[CamLight][Commands] - 'BOOT' button pushed!")
        self.states.set_state('camlight', 'validate', state=False, update_label_pixmap=self._update_label_enable)
