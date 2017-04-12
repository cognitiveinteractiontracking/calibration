// ROS
#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <atomic>


std::vector<tf::Pose> poses;
std::atomic<std::size_t> atomicCounter;
void callback(nav_msgs::OdometryConstPtr msg) {

  if (atomicCounter > 0) {
    tf::Pose poseCamera2Marker;
    tf::poseMsgToTF(msg->pose.pose, poseCamera2Marker);
    tf::Pose poseMarker2Camera = poseCamera2Marker.inverse();
    poses.at(atomicCounter-1) = poseMarker2Camera;
    --atomicCounter;
  }
}

static std::vector<ros::Publisher> pubOdom;
static ros::Subscriber sub;
int main(int argc, char** argv) {
  char glutGamemode[32];

  ros::init(argc, argv, "one_shot_calib_node");
  ros::NodeHandle n("~");

  std::string topic;
  int counter;
  n.param<std::string>("topic", topic, "/odom");
  n.param<int>("number_percepts_for_average", counter, 1);
  // Rounding to the next power of 2 for correct mean calculation
  counter = pow(2, ceil(log(counter)/log(2)));
  ROS_INFO_STREAM("Using " << counter << " percepts");

  poses.resize(counter);
  atomicCounter = counter;

  ROS_INFO_STREAM("Start listening on " << topic);
  ros::Subscriber sub = n.subscribe(topic, 1, callback);

  ros::Rate rate(100);
  while(ros::ok()) {
      if (!atomicCounter) {
          break;
      }
      ros::spinOnce();
      rate.sleep();
  }

  // Refinement
  if (poses.size() != 1) {
      for (int idx = poses.size(); idx > 1; idx /= 2) {
        int idSingle = 0, idDouble = 0;
        for (; idDouble < idx; idDouble += 2, idSingle += 1) {
//            ROS_INFO_STREAM("poses.at(idDouble).getOrigin().getX(): " << poses.at(idDouble).getOrigin().getX());
//            ROS_INFO_STREAM("poses.at(idDouble+1).getOrigin().getX(): " << poses.at(idDouble+1).getOrigin().getX());
            poses.at(idSingle).setOrigin(tf::Vector3(
                (poses.at(idDouble).getOrigin().getX() + poses.at(idDouble+1).getOrigin().getX())/2.0,
                (poses.at(idDouble).getOrigin().getY() + poses.at(idDouble+1).getOrigin().getY())/2.0,
                (poses.at(idDouble).getOrigin().getZ() + poses.at(idDouble+1).getOrigin().getZ())/2.0));
            tf::Quaternion posesTmp = poses.at(idDouble).getRotation();
            poses.at(idSingle).setRotation(posesTmp.slerp(poses.at(idDouble+1).getRotation(), 0.5));
//            ROS_INFO_STREAM("poses.at(idSingle).getOrigin().getX(): " << poses.at(idSingle).getOrigin().getX());
        }
        ROS_INFO_STREAM("Transformation from marker to camera coordinate system (X Y Z x y z w): " <<
                        poses.at(0).getOrigin().getX() << " " <<
                        poses.at(0).getOrigin().getY() << " " <<
                        poses.at(0).getOrigin().getZ() << " " <<
                        poses.at(0).getRotation().x() << " " <<
                        poses.at(0).getRotation().y() << " " <<
                        poses.at(0).getRotation().z() << " " <<
                        poses.at(0).getRotation().w());

      }
  }

  ROS_INFO_STREAM("FINAL: Transformation from marker to camera coordinate system (X Y Z x y z w): " <<
                  poses.at(0).getOrigin().getX() << " " <<
                  poses.at(0).getOrigin().getY() << " " <<
                  poses.at(0).getOrigin().getZ() << " " <<
                  poses.at(0).getRotation().x() << " " <<
                  poses.at(0).getRotation().y() << " " <<
                  poses.at(0).getRotation().z() << " " <<
                  poses.at(0).getRotation().w());

  return 0;
}
