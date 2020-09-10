#!/usr/bin/env python

import rospy
from decenter_fms_manager import DecenterFMSManager


def main():

    rospy.init_node("decenter_fms_manager")

    rc_node = DecenterFMSManager()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()