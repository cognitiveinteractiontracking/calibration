#include <ros/ros.h>
#include <tf/tf.h>
#include <boost/algorithm/string.hpp>
#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include <calib/TransformConfig.h>
#include <mutex>

std::mutex mtxReconf;
static tf::Transform transform;

void callback(calib::TransformConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request:\nx:\t%f\ny:\t%f\nz:\t%f\nroll:\t%f\npitch:\t%f\nyaw:\t%f",
            config.x, config.y, config.z,
            config.roll, config.pitch, config.yaw);
  mtxReconf.lock();
  transform = tf::Pose(
            tf::createQuaternionFromRPY(config.roll, config.pitch, config.yaw),
            tf::Vector3(config.x, config.y, config.z));
  mtxReconf.unlock();
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "dynamic_transform_publisher");
  ros::NodeHandle n("~");

  std::string tfEuler, tfQuat, targetFrameId, sourceFrameId, reconfTopic;
  double rate;
  const int tfEulerComponents = 6, tfQuatComponents = 7;
  std::vector<std::string> tfEulerVecStr, tfQuatVec;
  std::vector<double> tfEulerVecDbl(tfEulerComponents), tfQuatDbl(tfQuatComponents);
  tf::StampedTransform transformToBr;
  tf::TransformBroadcaster br;
  dynamic_reconfigure::Server<calib::TransformConfig> server;
  dynamic_reconfigure::Server<calib::TransformConfig>::CallbackType f;

  n.param<std::string>("tf_XYZrpy", tfEuler, ""); // (m,m,m,rad,rad,rad) In the form of "1.1 0 2.1 3.14 0 0"
  n.param<std::string>("tf_XYZxyzq", tfQuat, ""); // (m,m,m,1,1,1,1) In the form of "2 0 1 0 0 0 1"
  n.param<std::string>("source_frame", sourceFrameId, "");
  n.param<std::string>("target_frame", targetFrameId, "");
  n.param<double>("rate", rate, 1); // (Hz) Publish rate

  boost::split(tfEulerVecStr, tfEuler, boost::is_any_of(" "));
  boost::split(tfQuatVec, tfQuat, boost::is_any_of(" "));

  for (auto it = tfEulerVecStr.begin(); it != tfEulerVecStr.end(); ++it)
    ROS_ERROR_STREAM(*it);

  // Sanity checks
  if (!(!tfEuler.empty() != !tfQuat.empty())) {
      ROS_ERROR("tf_XYZrpy XOR tf_XYZxyzq needs to be given");
      return 1;
  }

  // Check with euler angles
  if (!tfEuler.empty()) {
    if (tfEulerVecStr.size() != tfEulerComponents) {
        ROS_ERROR("tf_XYZrpy components error (%d != %d). Maybe there are trailing spaces?",
                  int(tfEulerVecStr.size()), tfEulerComponents);
        return 1;
    } else {
        for (int idx = 0; idx < tfEulerComponents; ++idx) {
            tfEulerVecDbl.at(idx) = std::stod(tfEulerVecStr.at(idx));
        }
        transform = tf::Pose(
            tf::createQuaternionFromRPY(tfEulerVecDbl.at(3), tfEulerVecDbl.at(4), tfEulerVecDbl.at(5)),
            tf::Vector3(tfEulerVecDbl.at(0), tfEulerVecDbl.at(1), tfEulerVecDbl.at(2)));
        ROS_INFO("Use tf_XYZrpy with (x,y,z,r,p,y): %s", tfEuler.c_str());
    }
  }

  // Check with quaternions
  if (!tfQuat.empty()) {
    if (tfQuatVec.size() != tfQuatComponents) {
        ROS_ERROR("tf_XYZxyzq components error (%d != %d). Maybe there are trailing spaces?",
                  int(tfQuatVec.size()), tfQuatComponents);
        return 1;
    } else {
        for (int idx = 0; idx < tfEulerComponents; ++idx) {
            tfQuatDbl.at(idx) = std::stod(tfEulerVecStr.at(idx));
        }
        transform = tf::Pose(
            tf::Quaternion(tfQuatDbl.at(3), tfQuatDbl.at(4), tfQuatDbl.at(5), tfQuatDbl.at(6)),
            tf::Vector3(tfQuatDbl.at(0), tfQuatDbl.at(1), tfQuatDbl.at(2)));
        ROS_INFO("Use tf_XYZxyzq (x,y,z,qx,qy,qz,qw): %s", tfQuat.c_str());
    }
  }

  // Setup dynamic reconfiguration
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  // The tf broadcaster
  ros::Rate sleeper(rate);
  while (n.ok()) {
    mtxReconf.lock();
    transformToBr = tf::StampedTransform(transform, ros::Time::now(), sourceFrameId, targetFrameId);
    mtxReconf.unlock();
    br.sendTransform(transformToBr);
    sleeper.sleep();
  }
  return 0;
}
