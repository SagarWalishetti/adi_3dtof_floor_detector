/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#ifndef ADI_3DTOF_FLOOR_DETECTOR_EXAMPLE_H
#define ADI_3DTOF_FLOOR_DETECTOR_EXAMPLE_H

#include <compressed_depth_image_transport/compression_common.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/image.h>
#include <sensor_msgs/msg/point_cloud2.h>

#include <chrono>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "image_proc_utils.h"
#include "ros-perception/image_transport_plugins/compressed_depth_image_transport/rvl_codec.h"

using namespace std::chrono_literals;

#define MAX_QUEUE_SIZE_FOR_TIME_SYNC 2

/**
 * @brief This is main class for the ADI3DToFFloorDetectorExample node
 *
 *
 */
class ADI3DToFFloorDetectorExample : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new ADI3DToFFloorDetectorExample object
   *
   * @param cam_prefix Camera Prefix
   * @param enable_pointcloud_output Enable floor removed point cloud generation
   */
  ADI3DToFFloorDetectorExample()
  : Node("adi_3dtof_floor_detector_example"),
    sync_(
      sync_policy_depth_floor_(MAX_QUEUE_SIZE_FOR_TIME_SYNC), depth_image_subscriber_,
      floor_mask_subscriber_),
    sync_compressed_(
      sync_policy_compressed_depth_floor_(MAX_QUEUE_SIZE_FOR_TIME_SYNC),
      compressed_depth_image_subscriber_, compressed_floor_mask_subscriber_)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "ADI3DToF Floor Detector output generation::Inside ADI3DToFFloorDetectorExample()");

    //Parameters
    this->declare_parameter<std::string>("param_rostopic_cam_prefix", "cam1");
    cam_prefix_ = this->get_parameter("param_rostopic_cam_prefix").as_string();

    this->declare_parameter<bool>("param_enable_pointcloud_output", false);
    enable_pointcloud_output_ = this->get_parameter("param_enable_pointcloud_output").as_bool();

    // Topics : camera-info, depth, ir and floor mask
    camera_info_topic_name_ = "/" + cam_prefix_ + "/camera_info";
    depth_image_topic_name_ = "/" + cam_prefix_ + "/depth_image";
    // ir_image_topic_name_ = "/" + cam_prefix_ + "/ir_image";
    floor_mask_topic_name_ = "/" + cam_prefix_ + "/floor_mask";

    compressed_depth_image_topic_name_ = "/" + cam_prefix_ + "/depth_image/compressedDepth";
    // compressed_ir_image_topic_name_ = "/" + cam_prefix_ + "/ir_image/compressedDepth";
    compressed_floor_mask_topic_name_ = "/" + cam_prefix_ + "/compressed_floor_mask";

    camera_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/" + cam_prefix_ + "/camera_info", 1,
      std::bind(&ADI3DToFFloorDetectorExample::camInfoCallback, this, std::placeholders::_1));

    depth_image_subscriber_.subscribe(this, depth_image_topic_name_);
    // ir_image_subscriber_.subscribe(this, ir_image_topic_name_);
    floor_mask_subscriber_.subscribe(this, floor_mask_topic_name_);

    sync_.registerCallback(&ADI3DToFFloorDetectorExample::syncCallback, this);

    compressed_depth_image_subscriber_.subscribe(this, compressed_depth_image_topic_name_);
    // compressed_ir_image_subscriber_.subscribe(this, compressed_ir_image_topic_name_);
    compressed_floor_mask_subscriber_.subscribe(this, compressed_floor_mask_topic_name_);

    sync_compressed_.registerCallback(&ADI3DToFFloorDetectorExample::syncCompressedCallback, this);

    // Create TF listerner instance
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Flags for received topics
    camera_parameters_updated_ = false;
    depth_image_recvd_ = false;
    ir_image_recvd_ = false;
    floor_mask_image_recvd_ = false;

    // Images
    depth_image_ = nullptr;
    ir_image_ = nullptr;
    floor_mask_image_ = nullptr;
    xyz_image_ = nullptr;

    // Create publishers.
    floor_removed_depth_image_publisher_ =
      this->create_publisher<sensor_msgs::msg::Image>("floor_removed_depth_image", 10);
    floor_marked_depth_image_publisher_ =
      this->create_publisher<sensor_msgs::msg::Image>("floor_marked_depth_image", 10);
    if (enable_pointcloud_output_) {
      floor_removed_pointcloud_publisher_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("floor_removed_point_cloud", 10);
    }

    // init
    frame_counter_ = 0;

    timer_ =
      this->create_wall_timer(10ms, std::bind(&ADI3DToFFloorDetectorExample::generateOutput, this));
  }

  /**
   * @brief Destroy the ADI3DToFFloorDetectorExample object
   *
   */
  ~ADI3DToFFloorDetectorExample()
  {
    if (depth_image_ != nullptr) {
      delete[] depth_image_;
      depth_image_ = nullptr;
    }

    if (ir_image_ != nullptr) {
      delete[] ir_image_;
      ir_image_ = nullptr;
    }

    if (floor_mask_image_ != nullptr) {
      delete[] floor_mask_image_;
      floor_mask_image_ = nullptr;
    }

    if (xyz_image_ != nullptr) {
      delete[] xyz_image_;
      xyz_image_ = nullptr;
    }

    delete image_proc_utils_;
  }

  bool generateOutput();

private:
  unsigned int image_width_ = 512;
  unsigned int image_height_ = 512;
  std::string cam_prefix_;
  int frame_counter_;
  int enable_pointcloud_output_;
  bool camera_parameters_updated_;
  CameraIntrinsics camera_intrinsics_;
  ImageProcUtils * image_proc_utils_ = nullptr;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  sensor_msgs::msg::PointCloud2 transformed_pc_;

  std::string camera_info_topic_name_;
  std::string depth_image_topic_name_;
  std::string ir_image_topic_name_;
  std::string floor_mask_topic_name_;

  std::string compressed_depth_image_topic_name_;
  std::string compressed_ir_image_topic_name_;
  std::string compressed_floor_mask_topic_name_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_image_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::Image> ir_image_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::Image> floor_mask_subscriber_;

  message_filters::Subscriber<sensor_msgs::msg::CompressedImage> compressed_depth_image_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::CompressedImage> compressed_ir_image_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::CompressedImage> compressed_floor_mask_subscriber_;

  // sync policy
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image>
    sync_policy_depth_floor_;
  // synchronizer
  message_filters::Synchronizer<sync_policy_depth_floor_> sync_;

  // sync policy for compressed images
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage>
    sync_policy_compressed_depth_floor_;
  // synchronizer for compressed images
  message_filters::Synchronizer<sync_policy_compressed_depth_floor_> sync_compressed_;

  bool depth_image_recvd_;
  bool ir_image_recvd_;
  bool floor_mask_image_recvd_;

  unsigned short * depth_image_;
  unsigned short * ir_image_;
  unsigned char * floor_mask_image_;
  short * xyz_image_;

  rclcpp::Time stamp_;
  std::string frame_id_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr floor_removed_depth_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr floor_marked_depth_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr floor_removed_pointcloud_publisher_;

  rclcpp::Time curr_frame_timestamp_ = rclcpp::Clock{}.now();

  void camInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & cam_info);
  void depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & depth_image);
  void irImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & ir_image);
  void floorMaskImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & floor_mask_image);

  void syncCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_image,
    const sensor_msgs::msg::Image::ConstSharedPtr & floor_mask_image);

  void compressedDepthImageCallback(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_depth_image);
  void compressedIrImageCallback(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_ir_image);
  void compressedFloorMaskCallback(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_floor_mask_image);

  void syncCompressedCallback(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_depth_image,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_floor_mask_image);

  void generateFloorRemovedPointCloud();
  sensor_msgs::msg::PointCloud2::SharedPtr convert2ROSPointCloudMsg(
    short * xyz_frame, rclcpp::Time stamp, std::string frame_id);

  void getFloorDetectionOutput(cv::Mat & depth_image_op_16bit, cv::Mat & floor_detection_op_8bit);

  void publishImageAsRosMsg(
    const cv::Mat & img, const std::string & encoding_type, const std::string & frame_id,
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher);
};

#endif