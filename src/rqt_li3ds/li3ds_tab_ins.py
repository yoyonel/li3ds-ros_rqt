from li3ds_tabs import ILI3DSPlugin_Tabs


class LI3DSPlugin_INS(ILI3DSPlugin_Tabs):
    def __init__(self, *args, **kwargs):
        """

        :param args:
        :param kwargs:
        """
        super(LI3DSPlugin_INS, self).__init__(*args, **kwargs)

        # INS: Status
        self._sub_ins_status = self.ros.subscribe_to_topic(
            '/INS/SbgLogStatusData',
            self._cb_ins_status, queue_size=10
        )

        # ---------------------------------
        # ROS: Messages
        # ---------------------------------
        self._msg_ins_status = None

    @property
    def msg_status(self):
        return self._msg_ins_status

    def _cb_ins_status(self, msg):
        """

        :param msg:
        :type msg: SbgLogStatusData
        """
        # rospy.loginfo("_cb_ins_status- msg.generalStatus: {}".format(msg.generalStatus))
        self._msg_ins_status = msg
