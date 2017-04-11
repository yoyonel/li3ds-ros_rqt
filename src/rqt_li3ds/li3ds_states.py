from li3ds_tools import generate_id


class LI3DSPlugin_States(object):
    def __init__(self, li3ds_plugin):
        """

        :param li3ds_plugin:
        """

        self.li3ds_plugin = li3ds_plugin

        # dictionnaire des etats (GUI, Devices, ...)
        self._states = {}

    def _state_updated(self, device, command, update_label_pixmap=None, update_ros=False):
        """

        :param device:
        :param command:
        :param update_label_pixmap:
        :param update_ros:
        """
        # update GUI
        if update_label_pixmap:
            update_label_pixmap(device, command)
        # ROS
        if update_ros:
            self.li3ds_plugin.tab('arduino').state_updated()

    def get_state(self, device, command=''):
        """

        :param device:
        :param command:
        :return:
        """
        return self._states.get(generate_id(device, command), False)

    def set_state(self, device, command='', state=False,
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
        self._states[generate_id(device, command)] = state
        #
        self._state_updated(device, command, update_label_pixmap, update_ros)

    def switch_state(self, device, command='',
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
        self._states[generate_id(device, command)] ^= True
        self._state_updated(device, command, update_label_pixmap, update_ros)
        return self._states[generate_id(device, command)]

    def build_base_label_id(self, device, command=''):
        """

        :param device:
        :param command:
        :return:
        """
        return 'label_%s' % generate_id(device, command), self.get_state(device, command)
