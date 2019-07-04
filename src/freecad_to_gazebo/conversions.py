from math import radians as _radians


def deg2rad(d):
    '''Converts degrees to radians'''
    return _radians(d)

def flt2str(f):
    '''Converts floats to formatted string'''
    return '{:.6f}'.format(f)

