#!/usr/bin/env python

import sys
import rospy
from zstt.srv import MotionControl


def motion_control_client(motion, head_lr=-1, head_ud=-1, head_init=False):
    rospy.wait_for_service('motion_control')

    try:
        motion_control = rospy.ServiceProxy('motion_control', MotionControl)
        resp1 = motion_control(motion, head_lr, head_ud, head_init)
        return resp1.data
    except rospy.ServiceException, e:
        rospy.logerr('Service call failed: %s' % e)

def usage():
    return '%s [motion head_ud head_lr head_init]' % sys.argv[0]

if __name__ == '__main__':
    motion = ''
    head_lr = -1
    head_ud = -1
    head_init = False

    if len(sys.argv) > 1 and len(sys.argv) < 6:
        motion = str(sys.argv[1])
    else:
        print usage()
        sys.exit(1)

    if len(sys.argv) == 3:
        head_lr = int(sys.argv[2])
    if len(sys.argv) == 4:
        head_ud = int(sys.argv[3])
    if len(sys.argv) == 5:
        head_init = bool(sys.argv[4])

    print 'Requesting {}, {}, {}, {}'.format(motion, head_lr, head_ud, head_init)
    print 'Response {}'.format(motion_control_client(motion, head_lr, head_ud, head_init))
