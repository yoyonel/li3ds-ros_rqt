import sys
import subprocess
import signal
import os
import rostopic


def print_err(*args):
    sys.stderr.write(' '.join(map(str,args)) + '\n')
    sys.stderr.flush()


def update_gui_from_ros_msg(qt_tree, ros_msg, map_tree_to_ros_msg):
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


def iternodes(tree, call_getroot, call_getchildren):
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


def generate_id(*args):
    """

    :param args:
    :return:

    >>> generate_id()
    ''
    >>> generate_id('arduino', 'flash')
    'arduino_flash'
    >>> generate_id('arduino', '', 'flash')
    'arduino_flash'
    >>> generate_id('record')
    'record'
    >>> generate_id('record', '')
    'record'
    >>> generate_id('record', '', 'on')
    'record_on'
    """
    return "_".join(map(str, filter(None, args))).replace('__', '_')


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


def count_children_of_process(p):
    """

    :param p:
    :return:
    """
    ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % p.pid, shell=True, stdout=subprocess.PIPE)
    ps_output = ps_command.stdout.read()
    ps_command.wait()
    return len(ps_output.split("\n")[:-1])


def get_ros_master(*args):
    """

    :return:
    """
    # url: http://docs.ros.org/diamondback/api/rostopic/html/rostopic-pysrc.html
    return rostopic.rosgraph.masterapi.Master(*args)


def generate_list_rostopics(master):
    """

    :param master:
    :return:
    """
    return rostopic._master_get_topic_types(master)


class VirtualException(BaseException):
    def __init__(self, _type, _func):
        BaseException(self)
