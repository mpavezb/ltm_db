#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Author: Bhaskara Marthi

## %Tag(PYTHON_CLIENT)%

import roslib; roslib.load_manifest('ltm_db')
import rospy
import ltm_db as wr
import unittest
import math
import geometry_msgs.msg as gm

def make_pose(x, y, th):
    return gm.Pose(gm.Point(x, y, 0),
            gm.Quaternion(0, 0, math.sin(th/2), math.cos(th/2)))
    
def make_metadata(p, str):
    return {'x': p.position.x, 'y': p.position.y, 'name': str}

def to_tuple(p):
    return (p.position.x, p.position.y, p.position.z, p.orientation.x,
            p.orientation.y, p.orientation.z, p.orientation.w)

def eq_poses(p1, p2):
    return to_tuple(p1)==to_tuple(p2)
            

class TestWarehouseRosMongoPy(unittest.TestCase):

    def test_basic(self):

        # Set up collection
        coll = wr.MessageCollection("my_db", "poses", gm.Pose)
        p1 = make_pose(24, 42, 0)
        p2 = make_pose(10, 532, 3)
        p3 = make_pose(53, 22, 5)
        p4 = make_pose(22, -5, 33)

        # Insert pose objects with accompanying string metadata
        coll.insert(p1, make_metadata(p1, "bar"))
        coll.insert(p2, make_metadata(p2, "baz"))
        coll.insert(p3, make_metadata(p3, "qux"))
        coll.insert(p1, make_metadata(p1, "oof"))
        coll.insert(p4, make_metadata(p4, "ooof"))

        # Query poses s.t x < 40, y > ), in descending order of name
        results = coll.query({'x': {'$lt': 40}, 'y': {'$gt': 0}},\
                sort_by='name', ascending=False)

        # Turn list of pairs into pair of lists
        poses, metadata = zip(*list(results))
        
        self.assertEqual(len(poses), 3)
        self.assertTrue(eq_poses(p1, poses[0]))
        self.assertTrue(eq_poses(p2, poses[1]))
        self.assertTrue(eq_poses(p1, poses[2]))

        self.assertEqual(metadata[0]['name'], 'oof')
        self.assertEqual(metadata[1]['name'], 'baz')
        self.assertEqual(metadata[2]['name'], 'bar')

        # Update some messages
        coll.update(metadata[0], metadata={'name': 'bat'})
        coll.update(metadata[2], msg=p2)
        res2 = coll.query({'y': {'$gt': 40}}, sort_by='name')
        poses, metadata = zip(*list(res2))

        self.assertEqual(metadata[0]['name'], 'bar')
        self.assertEqual(metadata[1]['name'], 'bat')
        self.assertEqual(metadata[2]['name'], 'baz')
        self.assertTrue(eq_poses(p2, poses[0]))
        self.assertTrue(eq_poses(p1, poses[1]))
        self.assertTrue(eq_poses(p2, poses[2]))
        

        # Remove entries s.t. y<30
        self.assertEqual(5, coll.count())
        self.assertEqual(2, coll.remove({'y': {'$lt': 30}}))
        self.assertEqual(3, coll.count())

        # Test find_one
        self.assertTrue(coll.find_one({'y': {'$gt': 30}}))
        self.assertFalse(coll.find_one({'y': {'$lt': 30}}))
        

if __name__ == "__main__":
    rospy.init_node('test_ltm_db_mongo_py')
    import rostest
    rostest.rosrun('ltm_db_mongo', 'test_ltm_db_mongo_py', TestWarehouseRosMongoPy)

## %EndTag(PYTHON_CLIENT)%
