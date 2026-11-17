from _msgcore import make_message

Time = make_message('Time', [('sec', 'int32', int), ('nanosec', 'uint32', int)])
