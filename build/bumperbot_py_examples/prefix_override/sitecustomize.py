import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ofird/bumperbot_ws/install/bumperbot_py_examples'
