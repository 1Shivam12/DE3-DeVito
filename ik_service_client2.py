#!/usr/bin/env python

import argparse
import struct
import sys

import rospy

from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)

from std_msgs.msg import Header

from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest)


def ik_test(limb):
    rospy.init_node("rsdk_ik_service_client")
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'left': PoseStamped(header=hdr, pose=Pose(
                position=Point(
                    x=0.857579481614,
                    y=0.651981417433,
                    z=0.0388352386502),
                orientation=Quaternion(
                    x=-0.366894936773,
                    y=0.885980397775,
                    z=0.108155782462,
                    w=0.262162481772),
            ),
        ),
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.656982770038,
                    y=-0.852598021641,
                    z=0.0388609422173),
                orientation=Quaternion(
                    x=0.367048116303,
                    y=0.885911751787,
                    z=-0.108908281936,
                    w=0.261868353356),
            ),
        ),
    }

    ikreq.pose_stamp.append(poses[limb])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)

    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        print("\nSUCCESS - Valid Joint Solution Found from Seed Type: %s" %(seed_str))

        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print "\nIK Joint Solution:"
	print limb_joints.values()
        #print "------------------"
        #print "Response Message:\n", resp
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")

    return 0

def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
    parser.add_argument('-l', '--limb', choices=['left', 'right'], required=True, help="the limb to test")
    args = parser.parse_args(rospy.myargv()[1:])


    return ik_test(args.limb)

if __name__ == '__main__':
    sys.exit(main())
