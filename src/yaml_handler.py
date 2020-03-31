#!/usr/bin/env python
import rospy
import rosbag
import tf
import geometry_msgs.msg

class Sensor:
    def __init__(self, node_id, sensor_id, pos, quat, bag_location, ros_topic, node_name):
        self.node_id = node_id
        self.sensor_id = sensor_id
        self.pos = pos
        self.quat = quat
        self.bag_location = bag_location
        self.ros_topic = ros_topic
        self.node_name = node_name
        if len(pos) is not 3:
            print("Warning: Position of sensor "+self.id+" not completely described!")
        if len(quat) is not 4:
            print("Warning: Rotation of sensor "+self.id+" not completely described!")

    def info(self):
        print("Sensor "+str(self.sensor_id)+":")
        print("   node id = "+str(self.node_id))
        print("   pose: x="+str(self.pos[0])+" y="+str(self.pos[1])+
              " z="+str(self.pos[2]))
        print("   quaternion: qx="+str(self.quat[0])+" qy="+str(self.quat[1])+
              " qz="+str(self.quat[2])+" qw="+str(self.quat[3]))
        print("   bag_location = "+str(self.bag_location))
        print("   ros_topic = "+str(self.ros_topic))
        print("   node_name = "+str(self.node_name))
        print("\n")

class Tracking_Point:
    def __init__(self, node_id, tracker_id, sensor_id, pos, quat):
        self.node_id = node_id
        self.tracker_id = tracker_id
        self.sensor_id = sensor_id
        self.pos = pos
        self.quat = quat
        if len(pos) is not 3:
            print("Warning: Position of tracking point "+self.tracker_id+" not completely described!")
        if len(quat) is not 4:
            print("Warning: Rotation of tracking point "+self.tracker_id+" not completely described!")

    def info(self):
        print("Tracking Point "+str(self.tracker_id)+":")
        print("   node id = "+str(self.node_id))
        print("   cam_id = "+str(self.sensor_id))
        print("   pose: x="+str(self.pos[0])+" y="+str(self.pos[1])+
              " z="+str(self.pos[2]))
        print("   quaternion: qx="+str(self.quat[0])+" qy="+str(self.quat[1])+
              " qz="+str(self.quat[2])+" qw="+str(self.quat[3]))
        print("\n")

def check_id_num(input):
    output = ""
    for char in input:
        if ord(char) in range(48,58):
            output = output + str(char)
    return output

if __name__ == '__main__':
    rospy.init_node('yamlhandler')

    g2oFile = open("graph.gm2dl", "w+")

    sensor_list, pose, quat = [], [], []
    bag_loc, ros_topic, node_name = "", "", ""
    i = 1


    while True:
        try:
            pose.append(rospy.get_param("/s"+str(i)+"_pose/x"))
            pose.append(rospy.get_param("/s"+str(i)+"_pose/y"))
            pose.append(rospy.get_param("/s"+str(i)+"_pose/z"))
            quat.append(rospy.get_param("/s"+str(i)+"_pose/qx"))
            quat.append(rospy.get_param("/s"+str(i)+"_pose/qy"))
            quat.append(rospy.get_param("/s"+str(i)+"_pose/qz"))
            quat.append(rospy.get_param("/s"+str(i)+"_pose/qw"))

            bag_loc = rospy.get_param("/s"+str(i)+"_bag")
            ros_topic = rospy.get_param("/s"+str(i)+"_topic")
            node_name = rospy.get_param("/s"+str(i)+"_dyn_tf_node_name")

            sensor_list.append(Sensor(i-1, i, pose, quat, bag_loc, ros_topic, node_name))

            pose = []
            quat = []

        except KeyError:
            break

        i += 1

    tracking_points, node_list, unique_list = [], [], []
    bag_num = 1
    fixed_sensor = sensor_list[0].sensor_id

    for sensor in sensor_list:
        bag = rosbag.Bag(sensor.bag_location)
        print("Reading Bag "+str(bag_num)+"...")

        for topic, msg, t in bag.read_messages():
            # Only odometric messages
            if "odom" not in topic:
                continue

            tracker_id = check_id_num(msg.child_frame_id)

            # Only use the first entry of tracking point in rosbag  TODO: Use mean value over time?
            if tracker_id in unique_list:
                continue;

            cam_id = sensor.sensor_id
            pose.append(msg.pose.pose.position.x)
            pose.append(msg.pose.pose.position.y)
            pose.append(msg.pose.pose.position.z)
            quat.append(msg.pose.pose.orientation.x)
            quat.append(msg.pose.pose.orientation.y)
            quat.append(msg.pose.pose.orientation.z)
            quat.append(msg.pose.pose.orientation.w)

            tracking_points.append(Tracking_Point(i-1, tracker_id, cam_id, pose, quat))
            unique_list.append(tracker_id)

            pose = []
            quat = []

            if sensor.sensor_id is fixed_sensor:
                i += 1

        unique_list = []
        bag.close()

        bag_num += 1

    t = tf.TransformerROS(True, rospy.Duration(10.0))

    print("Writing in gm2dl file...")
    for snode in sensor_list:

        # Add sensor frames
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

        g2oFile.write("VERTEX_SE3:QUAT "+str(snode.node_id)+" "+str(snode.pos[0])+" "+
                      str(snode.pos[1])+" "+str(snode.pos[2])+" "+str(snode.quat[0])+" "+
                      str(snode.quat[1])+" "+str(snode.quat[2])+" "+str(snode.quat[3])+"\n")

        # Fixing values for first sensor
        if snode.sensor_id is fixed_sensor:
            g2oFile.write("FIX "+str(snode.node_id)+"\n")

    msg_point = geometry_msgs.msg.PoseStamped()
    trans = geometry_msgs.msg.PoseStamped()

    for tnode in tracking_points:

        if tnode.sensor_id is fixed_sensor:
            # Add tracker frames
            m = geometry_msgs.msg.TransformStamped()
            m.header.frame_id = "s"+str(tnode.sensor_id)
            m.child_frame_id = "t"+str(tnode.node_id)
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

            g2oFile.write("VERTEX_SE3:QUAT "+str(tnode.tracker_id)+" "+str(tpose.x)+" "+
                      str(tpose.y)+" "+str(tpose.z)+" "+str(tquat.x)+" "+
                      str(tquat.y)+" "+str(tquat.z)+" "+str(tquat.w)+"\n")
            g2oFile.write("FIX "+str(tnode.tracker_id)+"\n")

    fixed_cov = " 10000 0 0 0 0 0 10000 0 0 0 0 10000 0 0 0 40000 0 0 40000 0 40000 "
    valid_point_list = []

    for tpoint in tracking_points:
        if tpoint.sensor_id is fixed_sensor:
            valid_point_list.append(tpoint.tracker_id)

    for tpoint in tracking_points:
        if tpoint.tracker_id in valid_point_list:
            g2oFile.write("EDGE_SE3:QUAT "+str(tpoint.sensor_id-1)+" "+str(tpoint.tracker_id)+" "+
                      str(tpoint.pos[0])+" "+str(tpoint.pos[1])+" "+str(tpoint.pos[2])+" "+
                      str(tpoint.quat[0])+" "+str(tpoint.quat[1])+" "+str(tpoint.quat[2])+" "+
                      str(tpoint.quat[3])+fixed_cov+"\n")


    g2oFile.close()
    print("Done...")
