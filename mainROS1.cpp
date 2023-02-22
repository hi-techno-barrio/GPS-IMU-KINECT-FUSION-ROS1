#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <robot_localization/navsat_conversions.h>
#include <robot_localization/ros_filter_utilities.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

class RobotController
{
public:
  RobotController()
  {
    imu_subscriber_ = nh_.subscribe("/imu", 10, &RobotController::imuCallback, this);
    laser_subscriber_ = nh_.subscribe("/scan", 10, &RobotController::laserCallback, this);
    gps_subscriber_ = nh_.subscribe("/gps", 10, &RobotController::gpsCallback, this);
    kinect_subscriber_ = nh_.subscribe("/kinect/depth/points", 10, &RobotController::kinectCallback, this);
    motor_publisher_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    transform_tree_.setExtrapolationLimit(ros::Duration(0.1));
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
  {
    imu_data_ = *imu_msg;
  }

  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser_msg)
  {
    laser_data_ = *laser_msg;
  }

  void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
  {
    gps_data_ = *gps_msg;
  }

  void kinectCallback(const sensor_msgs::PointCloud2::ConstPtr& kinect_msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr kinect_data (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*kinect_msg, *kinect_data);
    kinect_data_ = *kinect_data;
  }

  void fuseSensorData()
  {
    // Fuse GPS and IMU data
    if (gps_data_.header.stamp == imu_data_.header.stamp)
    {
      double roll, pitch, yaw;
      tf::Quaternion quat;
      tf::quaternionMsgToTF(imu_data_.orientation, quat);
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

      Eigen::VectorXd measurement(6);
      measurement(0) = gps_data_.latitude;
      measurement(1) = gps_data_.longitude;
      measurement(2) = gps_data_.altitude;
      measurement(3) = imu_data_.linear_acceleration.x;
      measurement(4) = imu_data_.linear_acceleration.y;
      measurement(5) = yaw;

      Eigen::MatrixXd measurement_covariance(6, 6);
      measurement_covariance.setZero();
      measurement_covariance(0, 0) = gps_data_.position_covariance[0];
      measurement_covariance(1, 1) = gps_data_.position_covariance[4];
      measurement_covariance(2, 2) = gps_data_.position_covariance[8];
      measurement_covariance(3, 3) = imu_data_.linear_acceleration_covariance[0];
      measurement_covariance(4, 4) = imu_data_.linear_acceleration_covariance[4

      measurement_covariance(5, 5) = imu_data_.orientation_covariance[8];

      robot_filter_.setMeasurement(measurement);
      robot_filter_.setMeasurementCovariance(measurement_covariance);

      robot_filter_.processMeasurement(imu_data_.header.stamp.toSec());
    }

    // Fuse GPS and Kinect data
    if (gps_data_.header.stamp == kinect_data_.header.stamp)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr gps_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      robot_localization::NavsatConversions::LLAtoFlat(gps_data_.latitude, gps_data_.longitude, 0, gps_origin_.latitude, gps_origin_.longitude, 0, *gps_cloud);
      pcl::transformPointCloud(*gps_cloud, *gps_cloud, kinect_transform_);
      pcl::VoxelGrid<pcl::PointXYZ> downsample_filter;
      downsample_filter.setInputCloud(gps_cloud);
      downsample_filter.setLeafSize(0.1, 0.1, 0.1);
      downsample_filter.filter(*gps_cloud);

      pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
      icp.setInputSource(gps_cloud);
      icp.setInputTarget(kinect_data_.makeShared());

      pcl::PointCloud<pcl::PointXYZ> aligned_gps_cloud;
      icp.align(aligned_gps_cloud);

      Eigen::Matrix4f icp_transform = icp.getFinalTransformation();

      tf::Matrix3x3 rotation_matrix(icp_transform(0, 0), icp_transform(0, 1), icp_transform(0, 2), icp_transform(1, 0), icp_transform(1, 1), icp_transform(1, 2), icp_transform(2, 0), icp_transform(2, 1), icp_transform(2, 2));
      tf::Quaternion icp_quat;
      rotation_matrix.getRotation(icp_quat);

      tf::Transform icp_transform_tf;
      icp_transform_tf.setOrigin(tf::Vector3(icp_transform(0, 3), icp_transform(1, 3), icp_transform(2, 3)));
      icp_transform_tf.setRotation(icp_quat);

      transform_tree_.setTransform(tf::StampedTransform(icp_transform_tf, kinect_data_.header.stamp, "/gps", "/kinect"));

      sensor_msgs::PointCloud2 transformed_cloud;
      pcl_ros::transformPointCloud("kinect", kinect_data_, transformed_cloud, transform_tree_);

      pcl::PointCloud<pcl::PointXYZ>::Ptr kinect_data_aligned (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(transformed_cloud, *kinect_data_aligned);

      kinect_data_ = *kinect_data_aligned;
    }
  }

  void navigate()
  {
    // Generate trajectory using ROS Navigation stack
    // ...

    // Follow trajectory using PID controller or other methods
    // ...
  }

  void controlRobot()
  {
    // Fuse sensor data
    fuseSensorData();

    // Navigate using ROS Navigation stack
    navigate();

    // Control the robot's movement based on the navigation commands
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = navigation_linear_speed_;
    twist_msg.angular.z = navigation_angular_speed_;
    motor_publisher_.publish(twist_msg);
  }

  void initializeGpsOrigin()
  {
    robot_localization::NavsatConversions::LLAtoFlat(gps_data_.latitude, gps_data_.longitude, 0, gps_origin_.latitude, gps_origin_.longitude, gps_origin_.altitude,

    gps_origin_initialized_ = true;

    ROS_INFO_STREAM("GPS origin initialized to: " << gps_origin_.latitude << ", " << gps_origin_.longitude << ", " << gps_origin_.altitude);
  }

  void updateKinectTransform()
  {
    tf::StampedTransform transform;
    try
    {
      transform_tree_.lookupTransform("/gps", "/kinect", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("%s",ex.what());
      return;
    }

    kinect_transform_ = Eigen::Matrix4f::Identity();
    kinect_transform_(0, 0) = transform.getBasis()[0][0];
    kinect_transform_(0, 1) = transform.getBasis()[0][1];
    kinect_transform_(0, 2) = transform.getBasis()[0][2];
    kinect_transform_(1, 0) = transform.getBasis()[1][0];
    kinect_transform_(1, 1) = transform.getBasis()[1][1];
    kinect_transform_(1, 2) = transform.getBasis()[1][2];
    kinect_transform_(2, 0) = transform.getBasis()[2][0];
    kinect_transform_(2, 1) = transform.getBasis()[2][1];
    kinect_transform_(2, 2) = transform.getBasis()[2][2];
    kinect_transform_(0, 3) = transform.getOrigin().getX();
    kinect_transform_(1, 3) = transform.getOrigin().getY();
    kinect_transform_(2, 3) = transform.getOrigin().getZ();
  }

  void run()
  {
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
      if (!gps_origin_initialized_ && gps_data_.status.status >= sensor_msgs::NavSatStatus::STATUS_FIX)
      {
        initializeGpsOrigin();
      }

      if (kinect_data_.header.stamp != ros::Time(0))
      {
        updateKinectTransform();
      }

      controlRobot();

      ros::spinOnce();
      loop_rate.sleep();
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber laser_subscriber_;
  ros::Subscriber gps_subscriber_;
  ros::Subscriber kinect_subscriber_;
  ros::Publisher motor_publisher_;
  tf::TransformBroadcaster transform_broadcaster_;
  tf::TransformListener transform_tree_;
  sensor_msgs::Imu imu_data_;
  sensor_msgs::LaserScan laser_data_;
  sensor_msgs::NavSatFix gps_data_;
  pcl::PointCloud<pcl::PointXYZ> kinect_data_;
  Eigen::Matrix4f kinect_transform_;
  robot_localization::RosFilter<6> robot_filter_;
  sensor_msgs::NavSatFix gps_origin_;
  bool gps_origin_initialized_ = false;
  double navigation_linear_speed_;
  double navigation_angular_speed_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_controller");
  RobotController controller;

  controller.run();

  return 0;
}
