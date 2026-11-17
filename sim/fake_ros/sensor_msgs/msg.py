from _msgcore import make_message
from std_msgs.msg import Header

Image = make_message('Image', [
    ('header', 'std_msgs/Header', Header), ('height', 'uint32', int), ('width', 'uint32', int),
    ('encoding', 'string', str), ('is_bigendian', 'uint8', int), ('step', 'uint32', int),
    ('data', 'uint8[]', bytes)])
BatteryState = make_message('BatteryState', [
    ('header', 'std_msgs/Header', Header), ('voltage', 'float32', lambda: float('nan')),
    ('percentage', 'float32', lambda: float('nan'))])
