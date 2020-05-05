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
        self.id = int(id)
        self.pose = pose
        self.bag_location = bag_location
        self.ros_topic = ros_topic
        self.node_name = node_name


class Tracking_Point:
    def __init__(self, tracker_id, sensor_id, pose):
        self.tracker_id = int(tracker_id)
        self.sensor_id = int(sensor_id)
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
        trans, rot = [], []
        translation = self.vertex(id).estimate().translation()
        rotation = self.vertex(id).estimate().rotation()
        for i in range(3):
            trans.append(float(translation[i]))
        if with_euler: # Transform quat to euler angles in radians
            quaternion = (rotation.x(), rotation.y(), rotation.z(), rotation.w())
            euler = tf.transformations.euler_from_quaternion(quaternion)
            for i in range(3):
                rot.append(euler[i])
        else:
            rot.append(rotation.x())
            rot.append(rotation.y())
            rot.append(rotation.z())
            rot.append(rotation.w())
        return (trans, rot)

def check_id_num(input):
    output = ""
    for char in input:
        # Check if input is number in ASCII
        if ord(char) in range(48,58):
            output = output + str(char)
    return int(output)

def add_frame(transformator, cur_frame, parent_frame, pose):
    m = geometry_msgs.msg.TransformStamped()
    m.header.frame_id = parent_frame
    m.child_frame_id = cur_frame
    tran = m.transform.translation
    rot = m.transform.rotation
    tran.x, tran.y, tran.z = pose['x'], pose['y'], pose['z']
    rot.x, rot.y, rot.z, rot.w = pose['qx'], pose['qy'], pose['qz'], pose['qw']
    transformator.setTransform(m)

def transform_frame(transformator, cur_frame, target_frame, pose):
    trans = geometry_msgs.msg.PoseStamped()
    tf_pos = msg_point.pose.position
    tf_rot = msg_point.pose.orientation
    msg_point.header.frame_id = cur_frame
    tf_pos.x, tf_pos.y, tf_pos.z = pose['x'], pose['y'], pose['z']
    tf_rot.x, tf_rot.y, tf_rot.z, tf_rot.w =  pose['qx'], pose['qy'], pose['qz'], pose['qw']
    trans = t.transformPose(target_frame, msg_point)
    tpose = trans.pose.position
    tquat = trans.pose.orientation

    return (tpose, tquat)

def transform_to_SE3Quat(transformer, cur_frame, target_frame, pose):
    (tpose, tquat) = transform_frame(transformer, cur_frame, target_frame, pose)

    return g2o.SE3Quat(g2o.Quaternion(tquat.w, tquat.x, tquat.y, tquat.z),
                       np.array([tpose.x, tpose.y, tpose.z]))

def fill_pose_from_msg(translation, rotation):
    dict = {
        "x": translation.x, "y": translation.y, "z": translation.z,
        "qx": rotation.x, "qy": rotation.y, "qz": rotation.z,
        "qw": rotation.w
    }
    return dict


def fill_pose_from_list(translation, rotation, withEuler):
    if withEuler:
        dict = {
            "x": translation[0], "y": translation[1], "z": translation[2],
            "roll": rotation[0], "pitch": rotation[1], "yaw": rotation[2]
        }
    else:
        dict = {
            "x": translation[0], "y": translation[1], "z": translation[2],
            "qx": rotation[0], "qy": rotation[1], "qz": rotation[2],
            "qw": rotation[3]
        }
    return dict


if __name__ == '__main__':
    rospy.init_node('calibrator')
    optimizer = GraphOptimizer()
    t = tf.TransformerROS(True, rospy.Duration(10.0))

    sensor_list = []
    sensor_num, graph_id = 1, 1

    try:
        # Getting the calibration frame
        origin_frame = rospy.get_param("/calibration_frame_id") # Exp: tracking_base
    except KeyError:
        print("No calibration_frame_id found")

    try:
        calibration_marker_id = int(rospy.get_param("/calibration_marker_id")) # Tracker point
        markerIDset = True
    except KeyError:
        markerIDset = False
        print("calibration_marker_id not defined")

    # If calibration_offset is set, tracking_base and reference sensor shall be
    # set at that pose
    try:
        calibration_offset = rospy.get_param("/calibration_offset")
        offsetIsSet = True
    except KeyError:
        offsetIsSet = False
        print("calibration_offset not definded")

    if markerIDset and offsetIsSet:
        print("Warning: Cannot operate with both calibration_offset "+
              "AND calibration_marker_id. Using calibration_offset per default")
        markerIDset = False



    while True:
        try:
            pose = rospy.get_param("/s"+str(sensor_num)+"_pose")
            bag_loc = rospy.get_param("/s"+str(sensor_num)+"_bag")
            ros_topic = rospy.get_param("/s"+str(sensor_num)+"_topic")
            node_name = rospy.get_param("/s"+str(sensor_num)+"_dyn_tf_node_name")

            sensor_list.append(Sensor(sensor_num, pose, bag_loc, ros_topic, node_name))
            sensor_num += 1
            graph_id += 1
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
            pose = fill_pose_from_msg(position, orientation)

            tracking_points.append(Tracking_Point(tracker_id, cam_id, pose))
            unique_list.append(tracker_id)

        if cam_id is fixed_sensor and markerIDset:
            if calibration_marker_id not in unique_list:
                print("Warning - calibration_marker_id "+str(calibration_marker_id)+" is not seen by reference sensor.")
                print("Use one of these marker ids: "+str(unique_list))
                markerIDset = False

        unique_list = []
        bag_num += 1
        bag.close()

    msg_point = geometry_msgs.msg.PoseStamped()
    trans = geometry_msgs.msg.PoseStamped()

    # Create nodes in graph for sensors
    print("Generate graph for optimization...")
    for snode in sensor_list:
        spose = snode.pose
        add_frame(t, "s"+str(snode.id), origin_frame, spose)
        pose = g2o.SE3Quat(g2o.Quaternion(spose['qw'], spose['qx'], spose['qy'], spose['qz']),
                           np.array([spose['x'], spose['y'], spose['z']]))
        optimizer.add_vertex(snode.id, pose.Isometry3d(), snode.id is fixed_sensor)

    # Create nodes in graph for tracking points (transformed in origin frame)
    assocs = []
    graph_id = sensor_num + 1
    for tnode in tracking_points:
        if tnode.sensor_id is fixed_sensor:
            tpose = tnode.pose
            add_frame(t, "t"+str(tnode.tracker_id), "s"+str(tnode.sensor_id), tpose)

            pose = transform_to_SE3Quat(t, "s"+str(tnode.sensor_id), origin_frame, tpose)
            optimizer.add_vertex(graph_id, pose.Isometry3d())
            # To avoid id conflicts in g2o graph, graph_id not tracking_id for one tracking_point
            # graph_ids saves the association of these values
            assocs.append((graph_id, tnode.tracker_id))
            graph_id += 1

    # Creating edges between sensors and tracking points
    cov = np.array([[10000,0,0,0,0,0], [0,10000,0,0,0,0], [0,0,10000,0,0,0],
                    [0,0,0,40000,0,0], [0,0,0,0,40000,0], [0,0,0,0,0,40000]])
    for tpoint in tracking_points:
        for graph_id, tracker_id in assocs:
            if tpoint.tracker_id is tracker_id:
                tpose = tpoint.pose
                pose = g2o.SE3Quat(g2o.Quaternion(tpose['qw'], tpose['qx'], tpose['qy'], tpose['qz']),
                                   np.array([tpose['x'], tpose['y'], tpose['z']]))
                optimizer.add_edge([int(tpoint.sensor_id), graph_id], pose.Isometry3d(), cov)

    print("Done.")
    optimizer.optimize() # This is where the magical optimization happens

    # If calibration_frame_id is set, the transformation of tracking_base with
    # backtransformation to world frame of its children frames is executed
    if markerIDset or offsetIsSet:
        try:
            listener = tf.TransformListener()
            # Waiting until transform is available
            while not listener.canTransform("world", origin_frame, rospy.Time.now()):
                continue

            (transToWorld, rotToWorld) = listener.lookupTransform("world", origin_frame, rospy.Time.now())
            pose = fill_pose_from_list(transToWorld, rotToWorld, withEuler=False)
            add_frame(t, origin_frame, "world", pose)

            if offsetIsSet:
                tpose = calibration_offset

            if markerIDset:
                for tnode in tracking_points:
                    if tnode.tracker_id is calibration_marker_id and tnode.sensor_id is fixed_sensor:
                        tpose = tnode.pose

            (trans, rot) = transform_frame(t, "s"+str(fixed_sensor), "world", tpose)

            quaternion = (rot.x, rot.y, rot.z, rot.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)

            client = reconf.Client(rospy.get_param("/calibration_dyn_tf_node_name"))
            client.update_configuration({"x": trans.x, "y": trans.y, "z": trans.z,
                                         "roll": euler[0], "pitch": euler[1], "yaw": euler[2]})

            # cameras transformed back by same Offset
            (trans_offset, rot_offset) = transform_frame(t, "s"+str(fixed_sensor), origin_frame, tpose)
            offset = fill_pose_from_msg(trans_offset, rot_offset)
            add_frame(t, "new_origin", "world", offset)

            for snode in sensor_list:
                (c_trans, c_rot) = optimizer.get_pose(snode.id, False)
                cam_pose = fill_pose_from_list(c_trans, c_rot, withEuler=False)

                (trans, rot) = transform_frame(t, "world", "new_origin", cam_pose)
                quaternion = (rot.x, rot.y, rot.z, rot.w)
                euler = tf.transformations.euler_from_quaternion(quaternion)

                client = reconf.Client(snode.node_name)
                client.update_configuration({"x": trans.x, "y": trans.y, "z": trans.z,
                                             "roll": euler[0], "pitch": euler[1], "yaw": euler[2]})

        except tf.LookupException:
            print("LookUp failed. Node "+str(origin_frame)+" not transformable")

    # If calibration_marker_id is not set, it is assumend that world and tracking_base are equivalent
    else:
        for snode in sensor_list:
            (c_trans, c_rot) = optimizer.get_pose(snode.id)
            cam_pose = fill_pose_from_list(c_trans, c_rot, withEuler=True)
            client = reconf.Client(snode.node_name)
            client.update_configuration(cam_pose)
