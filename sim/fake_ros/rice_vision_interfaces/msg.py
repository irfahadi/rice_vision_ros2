"""Kelas message dibangun langsung dari rice_vision_interfaces/msg/*.msg di repo."""
import os

from _msgcore import load_msg_dir
from _registry import resolve

_MSG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..',
                                        'rice_vision_interfaces', 'msg'))
globals().update(load_msg_dir(_MSG_DIR, 'rice_vision_interfaces', resolve))
