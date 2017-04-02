import rospy
import sys

from li3ds_tools import get_ros_master, generate_list_rostopics

# ROS: Messages
from velodyne_configuration.msg import VLP16_StatusMessage
from velodyne_configuration.msg import VLP16_DiagnosticsMessage
#
from arduino_msgs.msg import commands
from arduino_msgs.msg import states
#
from sbg_driver.msg import SbgLogStatusData


class LI3DSPlugin_ROS(object):
    def __init__(self):
        """

        """
        # url: http://docs.ros.org/diamondback/api/rostopic/html/rostopic-pysrc.html
        self._master = get_ros_master('/rostopic')
        self._list_ros_topics = generate_list_rostopics(self._master)

    def find_publisher(
            self,
            pub_name_searched,
            queue_size=10,
            *args):
        """

        :param pub_name_searched:
        :type pub_name_searched: str
        :param queue_size:
        :type queue_size: int
        :param args:
        :type args: list
        :return:
        :rtype: (rospy.Publisher, genpy.Message)
        """
        ros_pub = None
        msg_name = 'None'
        try:
            pub_name, msg_name = self.search_ros_topic_message(pub_name_searched)
            rospy.logdebug('Publish to: %s with message type: %s', pub_name, msg_name)
            ros_pub = rospy.Publisher(pub_name, eval(msg_name), queue_size=queue_size, *args)
        except Exception as e:
            rospy.logwarn('_publish_to_topic(pub_name_searched=%s, *args=%s)', pub_name_searched, str(args))
            rospy.logwarn('Unexpected error: %s - error: %s',
                          sys.exc_info()[0], e)
        finally:
            return ros_pub, eval(msg_name)

    def subscribe_to_topic(
            self,
            pub_name_searched,
            callback,
            queue_size=10):
        """
        """
        ros_sub = None
        try:
            pub_name, pub_msg = self.search_ros_topic_message(pub_name_searched)
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

    def search_ros_topic_message(self, pub_name_searched, from_import=True):
        """
        """
        pub_name = None
        pub_msg = 'None'
        try:
            pub_filtered = filter(lambda topic_msg: pub_name_searched in topic_msg[0], self._list_ros_topics)[0]
            pub_name, pub_msg = pub_filtered
            if from_import:
                pub_msg = pub_msg.split('/')[-1]
        except:
            rospy.logerr('Unexpected error: %s', sys.exc_info()[0])
        finally:
            return pub_name, pub_msg
