#!/usr/bin/env python
import rospy
import dynamic_reconfigure.client as reconf
import rosbag
import tf
import geometry_msgs.msg
import numpy as np
import g2o


class Sensor:
    def __init__(self, id, pose, bag_location, ros_topic, node_name):
        self.id = id
        self.pose = pose
        self.bag_location = bag_location
        self.ros_topic = ros_topic
        self.node_name = node_name


class Tracking_Point:
    def __init__(self, tracker_id, sensor_id, pose):
        self.tracker_id = tracker_id
        self.sensor_id = sensor_id
        self.pose = pose


class GraphOptimizer(g2o.SparseOptimizer):
    def __init__(self):
        super(GraphOptimizer, self).__init__()
        solver = g2o.BlockSolverSE3(g2o.LinearSolverCholmodSE3())
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        super(GraphOptimizer, self).set_algorithm(solver)

    def optimize(self, max_iterations=20):
        super(GraphOptimizer, self).initialize_optimization()
        super(GraphOptimizer, self).optimize(max_iterations)

    def add_vertex(self, id, pose, fixed=False):
        v_se3 = g2o.VertexSE3()
        v_se3.set_id(id)
        v_se3.set_estimate(pose) # g2o.SE3Quat().Isometry3d()
        v_se3.set_fixed(fixed)   # Fixation of relation point of graph
        super(GraphOptimizer, self).add_vertex(v_se3)

    def add_edge(self, vertices, measurement, information):
        edge = g2o.EdgeSE3()
        for i, v in enumerate(vertices):
            if isinstance(v, int):
                v = self.vertex(v)
            edge.set_vertex(i, v)
        edge.set_measurement(measurement)
        edge.set_information(information)
        super(GraphOptimizer, self).add_edge(edge)

    def get_pose(self, id, with_euler=True):
        pose = []
        translation = self.vertex(id).estimate().translation()
        rotation = self.vertex(id).estimate().rotation()
        for i in range(3):
            pose.append(float(translation[i]))
        if with_euler: # Transform quat to euler angles in radians
            quaternion = (rotation.x(), rotation.y(), rotation.z(), rotation.w())
            euler = tf.transformations.euler_from_quaternion(quaternion)
            for i in range(3):
                pose.append(euler[i])
        else:
            pose.append(rotation.x())
            pose.append(rotation.y())
            pose.append(rotation.z())
            pose.append(rotation.w())
        return pose

def check_id_num(input):
    output = ""
    for char in input:
        if ord(char) in range(48,58):
            output = output + str(char)
    return output


if __name__ == '__main__':
    rospy.init_node('calibrator')
    optimizer = GraphOptimizer()
    t = tf.TransformerROS(True, rospy.Duration(10.0))

    sensor_list = []
    sensor_num = 1

    try:
        # Getting the calibration frame
        origin_frame = rospy.get_param("/calibration_frame_id")
    except KeyError:
        try:
            # If no calibration frame, create one with pose from calibration offset
            cpose = rospy.get_param("/calibration_offset")
            origin_frame = "calibration_offset"

            m = geometry_msgs.msg.TransformStamped()
            tran = m.transform.translation
            rot = m.transform.rotation
            m.header.frame_id = "world"
            m.child_frame_id = origin_frame
            tran.x, tran.y, tran.z = cpose['x'], cpose['y'], cpose['z']
            rot.x, rot.y, rot.z, rot.w = cpose['qx'], cpose['qy'], cpose['qz'], cpose['qw']
            t.setTransform(m)
        except KeyError:
            # If there is no calibration offset, assume there is no offset to world frame
            print("Warning - Could not define origin frame. Using default world frame instead")
            origin_frame = "world"

    while True:
        try:
            pose = rospy.get_param("/s"+str(sensor_num)+"_pose")
            bag_loc = rospy.get_param("/s"+str(sensor_num)+"_bag")
            ros_topic = rospy.get_param("/s"+str(sensor_num)+"_topic")
            node_name = rospy.get_param("/s"+str(sensor_num)+"_dyn_tf_node_name")

            sensor_list.append(Sensor(sensor_num, pose, bag_loc, ros_topic, node_name))
            sensor_num += 1
        except KeyError:
            sensor_num -= 1
            break

    tracking_points, node_list, unique_list = [], [], []
    bag_num = 1
    fixed_sensor = sensor_list[0].id # Assume the first sensor is the reference for graph

    # Reading the rosbags for all tracking points provided by tracker
    for sensor in sensor_list:
        bag = rosbag.Bag(sensor.bag_location)
        print("Reading Bag "+str(bag_num)+" ...")

        for topic, msg, _ in bag.read_messages():

            # Only odometric messages
            if sensor.ros_topic not in topic:
                continue

            tracker_id = check_id_num(msg.child_frame_id)

            # Only use the first entry of tracking point in rosbag
            # OPTIMIZE: Use mean value over time?
            if tracker_id in unique_list:
                continue

            cam_id = sensor.id
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            pose = {
                "x": position.x, "y": position.y, "z": position.z,
                "qx": orientation.x, "qy": orientation.y, "qz": orientation.z,
                "qw": orientation.w
            }

            tracking_points.append(Tracking_Point(tracker_id, cam_id, pose))
            unique_list.append(tracker_id)

        unique_list = []
        bag_num += 1
        bag.close()

    msg_point = geometry_msgs.msg.PoseStamped()
    trans = geometry_msgs.msg.PoseStamped()

    # Create nodes in graph for sensors
    print("Generate graph for optimization...")
    for snode in sensor_list:

        # Add sensor frame
        m = geometry_msgs.msg.TransformStamped()
        m.header.frame_id = origin_frame
        m.child_frame_id = "s"+str(snode.id)
        tran = m.transform.translation
        rot = m.transform.rotation
        spose = snode.pose
        tran.x, tran.y, tran.z = spose['x'], spose['y'], spose['z']
        rot.x, rot.y, rot.z, rot.w = spose['qx'], spose['qy'], spose['qz'], spose['qw']
        t.setTransform(m)

        pose = g2o.SE3Quat(g2o.Quaternion(rot.w, rot.x, rot.y, rot.z),
                           np.array([tran.x, tran.y, tran.z]))
        optimizer.add_vertex(snode.id, pose.Isometry3d(), snode.id is fixed_sensor)



    # Create nodes in graph for tracking points (transformed in origin frame)
    for tnode in tracking_points:
        if tnode.sensor_id is fixed_sensor:
            # Add tracker frame
            m = geometry_msgs.msg.TransformStamped()
            m.header.frame_id = "s"+str(tnode.sensor_id)
            m.child_frame_id = "t"+str(tnode.tracker_id)
            tran = m.transform.translation
            rot = m.transform.rotation
            tpose = tnode.pose
            tran.x, tran.y, tran.z = tpose['x'], tpose['y'], tpose['z']
            rot.x, rot.y, rot.z, rot.w = tpose['qx'], tpose['qy'], tpose['qz'], tpose['qw']
            t.setTransform(m)

            # Convert tracking point from list to correct geometry_msgs.msg
            tf_pos = msg_point.pose.position
            tf_rot = msg_point.pose.orientation

            msg_point.header.frame_id = "s"+str(tnode.sensor_id)

            tf_pos.x, tf_pos.y, tf_pos.z = tran.x, tran.y, tran.z
            tf_rot.x, tf_rot.y, tf_rot.z, tf_rot.w =  rot.x, rot.y, rot.z, rot.w

            # Transform tracking point in sensor frame to origin frame
            trans = t.transformPose(origin_frame, msg_point)

            tpose = trans.pose.position
            tquat = trans.pose.orientation

            pose = g2o.SE3Quat(g2o.Quaternion(tquat.w, tquat.x, tquat.y, tquat.z),
                               np.array([tpose.x, tpose.y, tpose.z]))

            optimizer.add_vertex(int(tnode.tracker_id), pose.Isometry3d())


    # Collect the ids of tracking points seen by the fixed sensor
    valid_point_list = []
    for tpoint in tracking_points:
        if tpoint.sensor_id is fixed_sensor:
            valid_point_list.append(tpoint.tracker_id)

    # Fixed covariance for edges # OPTIMIZE: Calculate cov from tracker?
    cov = np.array([[10000,0,0,0,0,0],
                    [0,10000,0,0,0,0],
                    [0,0,10000,0,0,0],
                    [0,0,0,40000,0,0],
                    [0,0,0,0,40000,0],
                    [0,0,0,0,0,40000]])

    # Creating edges between sensors and tracking points
    for tpoint in tracking_points:
        if tpoint.tracker_id in valid_point_list:
            tpose = tpoint.pose
            pose = g2o.SE3Quat(g2o.Quaternion(tpose['qw'], tpose['qx'], tpose['qy'], tpose['qz']),
                               np.array([tpose['x'], tpose['y'], tpose['z']]))
            optimizer.add_edge([int(tpoint.sensor_id), int(tpoint.tracker_id)],
                                pose.Isometry3d(), cov)
    print("Done.")
    optimizer.optimize() # This is where the magical optimization happens

    for sensor in sensor_list:
        client = reconf.Client(sensor.node_name)
        c = optimizer.get_pose(sensor.id)
        client.update_configuration({"x": c[0], "y": c[1], "z": c[2],
                                     "roll": c[3], "pitch": c[4], "yaw": c[5]})
