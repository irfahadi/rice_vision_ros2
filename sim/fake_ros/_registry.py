"""Resolver nama tipe ROS 'pkg/Type' -> kelas message palsu."""

import importlib


def resolve(rtype):
    rtype = rtype.rstrip('[]')
    pkg, name = rtype.split('/')
    mod = importlib.import_module(f'{pkg}.msg')
    return getattr(mod, name)
