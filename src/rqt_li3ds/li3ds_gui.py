import rospy
import sys


class LI3DSPlugin_GUI(object):
    def __init__(self, widget, li3ds_states):
        """

        :param widget:
        """
        self._widget = widget
        #
        self._li3ds_states = li3ds_states

    @property
    def widget(self):
        return self._widget

    def update_label_pixmap_onoff(self, device, command=''):
        """

        :param device: ex: 'arduino'
        :type device: str
        :param command: ex: 'flash'
        :type command: str
        """
        label_id, state = self._li3ds_states.build_base_label_id(device, command)
        label_id += '_pixmap'
        #
        label_id_on = '%s_%s' % (label_id, 'on')
        label_id_off = '%s_%s' % (label_id, 'off')
        # rospy.loginfo("id_label_for_pixmap: %s - %s" % (label_id_on, label_id_off))
        try:
            self._widget.__dict__.get(label_id_on, None).setEnabled(state)
            self._widget.__dict__.get(label_id_off, None).setEnabled(not state)

            # urls:
            # - http://stackoverflow.com/questions/20866193/create-a-glowing-border-in-qss
            # - http://doc.qt.io/qt-4.8/qgraphicsblureffect.html
            # - http://doc.qt.io/qt-4.8/qtgui-module.html
            # if state:
            #     button = self._widget.__dict__.get(label_id_on, None)
            # else:
            #     button = self._widget.__dict__.get(label_id_off, None)
            # if button:
            #     # effect = QGraphicsDropShadowEffect(button)
            #     effect = QGraphicsBlurEffect(button)
            #     # effect.setOffset(0, 0)
            #     effect.setBlurRadius(5)
            #     button.setGraphicsEffect(effect)

        except AttributeError as e:
            rospy.logerr('_update_label_pixmap_onoff(%s, %s)' % (device, command))
            rospy.logerr('Unexpected error: %s - error: %s',
                         sys.exc_info()[0], e)
        finally:
            return label_id, state

    def update_label_enable(self, device, command=''):
        """

        :param device: ex: 'arduino'
        :type device: str
        :param command: ex: 'flash'
        :type command: str
        """
        #
        id_label, state = self._li3ds_states.build_base_label_id(device, command)
        id_label += '_pixmap'
        #
        # rospy.loginfo("id label for pixmap: %s" % id_label)
        try:
            self._widget.__dict__.get(id_label, None).setEnabled(state)
        except AttributeError as e:
            rospy.logerr('_update_label_enable(%s, %s)' % (device, command))
            rospy.logerr('Unexpected error: %s - error: %s', sys.exc_info()[0], e)
