#!/usr/bin/env python

# url: http://answers.ros.org/question/124706/how-to-run-a-newly-created-rqt-plugin/

import sys

from rqt_gui.main import Main

if __name__ == '__main__':
    main = Main()
    sys.exit(main.main(sys.argv, standalone='rqt_li3ds'))
