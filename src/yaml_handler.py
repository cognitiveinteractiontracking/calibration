#!/usr/bin/env python
import rospy
import dynamic_reconfigure.client as reconf
import rosbag
import tf
import geometry_msgs.msg
import numpy as np
import g2o


class Sensor:
    def __init__(self, sensor_id, pos, quat, bag_location, ros_topic, node_name):
        self.sensor_id = sensor_id
        self.pos = pos
        self.quat = quat
        self.bag_location = bag_location
        self.ros_topic = ros_topic
        self.node_name = node_name


class Tracking_Point:
    def __init__(self, tracker_id, sensor_id, pos, quat):
        self.tracker_id = tracker_id
        self.sensor_id = sensor_id
        self.pos = pos
        self.quat = quat


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
        for i in range(3):
            pose.append(self.vertex(id).estimate().translation()[i])
        if with_euler: # Transform quat to euler angles in radians
            quaternion = (self.vertex(id).estimate().rotation().x(),
                          self.vertex(id).estimate().rotation().y(),
                          self.vertex(id).estimate().rotation().z(),
                          self.vertex(id).estimate().rotation().w())
            euler = tf.transformations.euler_from_quaternion(quaternion)
            for i in range(3):
                pose.append(euler[i])
        else:
            pose.append(self.vertex(id).estimate().rotation().x())
            pose.append(self.vertex(id).estimate().rotation().y())
            pose.append(self.vertex(id).estimate().rotation().z())
            pose.append(self.vertex(id).estimate().rotation().w())
        return pose


def check_id_num(input):
    output = ""
    for char in input:
        if ord(char) in range(48,58):
            output = output + str(char)
    return output


if __name__ == '__main__':
    rospy.init_node('yamlhandler')
    optimizer = GraphOptimizer()

    sensor_list, pose, quat = [], [], []
    bag_loc, ros_topic, node_name = "", "", ""
    sensor_num = 1

    while True:
        try:
            pose.append(rospy.get_param("/s"+str(sensor_num)+"_pose/x"))
            pose.append(rospy.get_param("/s"+str(sensor_num)+"_pose/y"))
            pose.append(rospy.get_param("/s"+str(sensor_num)+"_pose/z"))
            quat.append(rospy.get_param("/s"+str(sensor_num)+"_pose/qx"))
            quat.append(rospy.get_param("/s"+str(sensor_num)+"_pose/qy"))
            quat.append(rospy.get_param("/s"+str(sensor_num)+"_pose/qz"))
            quat.append(rospy.get_param("/s"+str(sensor_num)+"_pose/qw"))

            bag_loc = rospy.get_param("/s"+str(sensor_num)+"_bag")
            ros_topic = rospy.get_param("/s"+str(sensor_num)+"_topic")
            node_name = rospy.get_param("/s"+str(sensor_num)+"_dyn_tf_node_name")

            sensor_list.append(Sensor(sensor_num, pose, quat, bag_loc, ros_topic, node_name))
            pose, quat = [], []
            sensor_num += 1
        except KeyError:
            sensor_num -= 1
            break

    tracking_points, node_list, unique_list = [], [], []
    bag_num = 1
    fixed_sensor = sensor_list[0].sensor_id # Assume the first sensor is the reference for graph

    # Reading the rosbags for all tracking points provided by tracker
    for sensor in sensor_list:
        bag = rosbag.Bag(sensor.bag_location)
        print("Reading Bag "+str(bag_num)+" ...")

        for topic, msg, t in bag.read_messages():
            # Only odometric messages
            if "odom" not in topic:
                continue

            tracker_id = check_id_num(msg.child_frame_id)

            # Only use the first entry of tracking point in rosbag  # OPTIMIZE: Use mean value over time?
            if tracker_id in unique_list:
                continue

            cam_id = sensor.sensor_id
            pose.append(msg.pose.pose.position.x)
            pose.append(msg.pose.pose.position.y)
            pose.append(msg.pose.pose.position.z)
            quat.append(msg.pose.pose.orientation.x)
            quat.append(msg.pose.pose.orientation.y)
            quat.append(msg.pose.pose.orientation.z)
            quat.append(msg.pose.pose.orientation.w)

            tracking_points.append(Tracking_Point(tracker_id, cam_id, pose, quat))
            unique_list.append(tracker_id)
            pose, quat = [], []

        unique_list = []
        bag_num += 1
        bag.close()

    t = tf.TransformerROS(True, rospy.Duration(10.0))
    optimizer_ids = []

    # Create nodes in graph for sensors
    print("Generate graph for optimization...")
    for snode in sensor_list:
        if snode.sensor_id is fixed_sensor:
            # Add sensor frame
            m = geometry_msgs.msg.TransformStamped()
            m.header.frame_id = "world"
            m.child_frame_id = "s"+str(snode.sensor_id)
            m.transform.translation.x = snode.pos[0]
            m.transform.translation.y = snode.pos[1]
            m.transform.translation.z = snode.pos[2]
            m.transform.rotation.x = snode.quat[0]
            m.transform.rotation.y = snode.quat[1]
            m.transform.rotation.z = snode.quat[2]
            m.transform.rotation.w = snode.quat[3]
            t.setTransform(m)

        squat = g2o.Quaternion(snode.quat[3],snode.quat[0],snode.quat[1],snode.quat[2])
        spos = np.array([snode.pos[0],snode.pos[1],snode.pos[2]])
        pose = g2o.SE3Quat(squat, spos)

        optimizer.add_vertex(snode.sensor_id, pose.Isometry3d(), snode.sensor_id is fixed_sensor)
        optimizer_ids.append(int(snode.sensor_id))

    msg_point = geometry_msgs.msg.PoseStamped()
    trans = geometry_msgs.msg.PoseStamped()

    # Create nodes in graph for tracking points (transformed in world frame)
    for tnode in tracking_points:
        if tnode.sensor_id is fixed_sensor:
            # Add tracker frame
            m = geometry_msgs.msg.TransformStamped()
            m.header.frame_id = "s"+str(tnode.sensor_id)
            m.child_frame_id = "t"+str(tnode.tracker_id)
            m.transform.translation.x = tnode.pos[0]
            m.transform.translation.y = tnode.pos[1]
            m.transform.translation.z = tnode.pos[2]
            m.transform.rotation.x = tnode.quat[0]
            m.transform.rotation.y = tnode.quat[1]
            m.transform.rotation.z = tnode.quat[2]
            m.transform.rotation.w = tnode.quat[3]
            t.setTransform(m)

            # Convert tracking point from list to correct geometry_msgs.msg
            msg_point.header.frame_id = "s"+str(tnode.sensor_id)
            msg_point.pose.position.x = tnode.pos[0]
            msg_point.pose.position.y = tnode.pos[1]
            msg_point.pose.position.z = tnode.pos[2]
            msg_point.pose.orientation.x = tnode.quat[0]
            msg_point.pose.orientation.y = tnode.quat[1]
            msg_point.pose.orientation.z = tnode.quat[2]
            msg_point.pose.orientation.w = tnode.quat[3]

            # Transform tracking point in sensor frame to world frame
            trans = t.transformPose("world", msg_point)

            tpose = trans.pose.position
            tquat = trans.pose.orientation

            tquat = g2o.Quaternion(tquat.w, tquat.x, tquat.y, tquat.z)
            tpos = np.array([tpose.x, tpose.y, tpose.z])
            pose = g2o.SE3Quat(tquat, tpos)

            optimizer.add_vertex(int(tnode.tracker_id), pose.Isometry3d())
            optimizer_ids.append(int(tnode.tracker_id))


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
            tquat = g2o.Quaternion(tpoint.quat[3], tpoint.quat[0], tpoint.quat[1],
                                   tpoint.quat[2])
            tpos = np.array([tpoint.pos[0], tpoint.pos[1], tpoint.pos[2]])
            pose = g2o.SE3Quat(tquat, tpos)
            optimizer.add_edge([int(tpoint.sensor_id), int(tpoint.tracker_id)],
                                pose.Isometry3d(), cov)
    print("Done...\n")
    optimizer.optimize() # This is where the magical optimization happens

    # Update camera poses via dynamic reconfiguration
    for i in optimizer_ids[:sensor_num]:
        client = reconf.Client("tf_cam"+str(i)+"_2")
        c = optimizer.get_pose(i)
        client.update_configuration({"x": c[0], "y": c[1], "z": c[2],
                                      "roll": c[3], "pitch": c[4], "yaw": c[5]})
