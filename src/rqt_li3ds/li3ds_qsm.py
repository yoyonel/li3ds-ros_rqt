from PyQt4.QtCore import QStateMachine, QState, pyqtSignal, QObject
from li3ds_tools import generate_id, print_err
import sys
from operator import itemgetter


class ILI3DSPlugin_QSM(QObject):
    """

    """
    def __init__(self,
                 device, command,
                 qt_widget,
                 initial_state='undefined'):
        """

        :param device:
        :type device: str
        :param command:
        :type command: str
        :param qt_widget:
        :type qt_widget:
        :param initial_state:
        :type initial_state: str
        """
        super(ILI3DSPlugin_QSM, self).__init__()

        self._widget = qt_widget
        #
        self._id = generate_id(device, command)
        #
        self._qsm = QStateMachine()
        #
        self._states = {}
        #
        self._add_states()
        self._add_transitions()
        #
        # -- Initial State
        self._qsm.setInitialState(self._get_state(initial_state))
        # - Launch QSM
        self._qsm.start()

    @property
    def _iter_strs_from_signals(self):
        """

        :return:
        """
        list_str_signals_transitions = map(
            lambda s: 'self.' + s,
            map(itemgetter(0), filter(lambda t: 'qsm_signals_' in t[0], LI3DSPlugin_QSM.__dict__.iteritems()))
        )
        for str_signal_transition in list_str_signals_transitions:
            state_start_name, state_end_name = str_signal_transition.split('self._qsm_signals_')[1].split('_to_')
            yield str_signal_transition, state_start_name, state_end_name

    def _add_states(self):
        """

        :return:
        """
        states_names = set()
        for _, state_start_name, state_end_name in self._iter_strs_from_signals:
            states_names.add(state_start_name)
            states_names.add(state_end_name)

        for state_name in states_names:
            self._add_state(state_name)

    def _add_transitions(self):
        """

        """
        # Add transitions from static declaration of list of signals used by QSM transitions.
        # it's a bit tricky but i can't see a simple solution to do this ...
        for str_signal_transition, state_start_name, state_end_name in self._iter_strs_from_signals:
            state_start, state_end = self._get_states((state_start_name, state_end_name))
            state_start.addTransition(
                eval(str_signal_transition),
                state_end
            )

    def _get_states(self, *args):
        """

        :param args:
        :return:

        >>> qsm = LI3DSPlugin_QSM('arduino', 'flash', None)
        >>> map(type, qsm._get_states(('undefined', 'on', 'off')))
        [<class 'PyQt4.QtCore.QState'>, <class 'PyQt4.QtCore.QState'>, <class 'PyQt4.QtCore.QState'>]
        """
        return map(self._get_state, *args)

    def _get_transition(self, state_start_name, state_end_name):
        """

        :param state_start_name:
        :type state_end_name: str
        :param state_end_name:
        :type state_end_name: str
        :return:
        :rtype: pyqtSignal

        # >>> qsm = LI3DSPlugin_QSM('arduino', 'flash', None)
        # >>> type(qsm._get_transition('undefined', 'off'))
        # <type 'PyQt4.QtCore.pyqtSignal'>
        """
        return self.__class__.__dict__.get(
            '_qsm_signals_%s_to_%s' % (state_start_name, state_end_name)
        )

    def _action_widget_textEdit(self, state_name):
        """

        :return:
        """
        try:
            widget_textEdit = self._widget.get(generate_id('textEdit', self._id))
            self._get_state(state_name).assignProperty(widget_textEdit, 'text', state_name)
        except AttributeError as e:
            print_err('Unexpected error: %s - error: %s', sys.exc_info()[0], e)

    def _add_state(self,
                   state_name,
                   state_action=_action_widget_textEdit):
        """

        :param state_name:
        :param state_action:
        :return:
        """
        id_state = self._generate_id_for_state(state_name)
        state = self._states[id_state] = QState()
        state.setObjectName(id_state)
        #
        state_action(self, state_name)
        # Add State to Machine
        self._qsm.addState(state)

    def _get_state(self, state_name):
        """

        :param state_name:
        :return:
        """
        return self._states[self._generate_id_for_state(state_name)]

    def _generate_id_for_state(self, state_name):
        """

        :param state_name:
        :return:

        >>> qsm = LI3DSPlugin_QSM('arduino', 'flash', None)
        >>> qsm._generate_id_for_state('undefined')
        'arduino_flash_undefined'
        """
        return generate_id(self._id, state_name)


class LI3DSPlugin_QSM(ILI3DSPlugin_QSM):
    # Signals for Transitions
    # need to be static and explicitly define ...
    _qsm_signals_undefined_to_on = pyqtSignal()
    _qsm_signals_undefined_to_off = pyqtSignal()
    _qsm_signals_off_to_on = pyqtSignal()
    _qsm_signals_on_to_off = pyqtSignal()

    def __init__(self, *args):
        """

        :param args:

        >>> expected_result = [ ('self._qsm_signals_off_to_on', 'off', 'on'),       \
                        ('self._qsm_signals_on_to_off', 'on', 'off'),               \
                        ('self._qsm_signals_undefined_to_off', 'undefined', 'off'), \
                        ('self._qsm_signals_undefined_to_on', 'undefined', 'on') ]
        >>> qsm = LI3DSPlugin_QSM('arduino', 'flash', None)
        >>> result = list(qsm._iter_strs_from_signals)
        >>> sorted(expected_result) == sorted(result)
        True
        """
        super(LI3DSPlugin_QSM, self).__init__(*args)
