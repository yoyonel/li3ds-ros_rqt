from PyQt4.QtCore import QObject


class ILI3DSPlugin_Tabs(QObject):
    # def __init__(self, li3ds_plugin):
    def __init__(self, *args, **kwargs):
        """

        :param args:
        :param kwargs:
        """
        super(ILI3DSPlugin_Tabs, self).__init__()

        self._states = kwargs.get('li3ds_states', None)
        self._gui = kwargs.get('li3ds_gui', None)
        self._ros = kwargs.get('li3ds_ros', None)

        self._li3ds_plugin = kwargs.get('li3ds_plugin', None)

    @property
    def states(self):
        return self._states

    @property
    def gui(self):
        return self._gui

    @property
    def ros(self):
        return self._ros

    def loginfo(self, *args):
        """

        :param args:
        :return:
        """
        return self._li3ds_plugin.loginfo(*args)
