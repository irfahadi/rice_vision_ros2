from _msgcore import make_message
from builtin_interfaces.msg import Time

String = make_message('String', [('data', 'string', str)])
Int32 = make_message('Int32', [('data', 'int32', int)])
Empty = make_message('Empty', [])
Header = make_message('Header', [('stamp', 'builtin_interfaces/Time', Time),
                                 ('frame_id', 'string', str)])
