/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#ifndef ADI_3DTOF_FLOOR_DETECTOR_NODE_H
#define ADI_3DTOF_FLOOR_DETECTOR_NODE_H

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/image.h>
#include <sensor_msgs/msg/point_cloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <chrono>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>

#include "adi_3dtof_floor_detector_output_info.h"
#include "adtf31xx_sensor_frame_info.h"
#include "floor_plane_detection.h"
#include "image_proc_utils.h"
#include "input_sensor.h"
#include "input_sensor_factory.h"
#include "output_sensor.h"
#include "output_sensor_factory.h"

using namespace std::chrono_literals;
namespace enc = sensor_msgs::image_encodings;

/**
 * @brief This is main class for this package
 *
 */
class ADI3DToFFloorDetector : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new ADI3DToFFloorDetector object
   */
  ADI3DToFFloorDetector() : Node("adi_3dtof_floor_detector_node")
  {
    // Declare Parameters

    //Read Only Parameters
    rcl_interfaces::msg::ParameterDescriptor base_camera_link_descriptor{};
    base_camera_link_descriptor.read_only = true;
    base_camera_link_descriptor.description = "Base camera frame name";
    camera_link_ = this->declare_parameter<std::string>(
      "param_camera_link", "adi_camera_link", base_camera_link_descriptor);

    rcl_interfaces::msg::ParameterDescriptor optical_camera_link_descriptor{};
    optical_camera_link_descriptor.read_only = true;
    optical_camera_link_descriptor.description = "Optical camera frame name";
    this->declare_parameter<std::string>(
      "param_optical_camera_link", "optical_camera_link", optical_camera_link_descriptor);

    rcl_interfaces::msg::ParameterDescriptor input_sensor_mode_descriptor{};
    input_sensor_mode_descriptor.read_only = true;
    input_sensor_mode_descriptor.description = "Input mode: 0:Camera, 2:FileIO";
    this->declare_parameter<int>("param_input_sensor_mode", 0, input_sensor_mode_descriptor);

    rcl_interfaces::msg::ParameterDescriptor output_sensor_mode_descriptor{};
    output_sensor_mode_descriptor.read_only = true;
    output_sensor_mode_descriptor.description =
      "Output mode: 0:no output is stored, 1:avi and csv file generated";
    this->declare_parameter<int>("param_output_sensor_mode", 0, output_sensor_mode_descriptor);

    rcl_interfaces::msg::ParameterDescriptor input_file_name_or_ros_topic_prefix_descriptor{};
    input_file_name_or_ros_topic_prefix_descriptor.read_only = true;
    input_file_name_or_ros_topic_prefix_descriptor.description =
      "Input file name in FileIO mode or rostopic prefix name";
    this->declare_parameter<std::string>(
      "param_input_file_name_or_ros_topic_prefix_name", "no name",
      input_file_name_or_ros_topic_prefix_descriptor);

    rcl_interfaces::msg::ParameterDescriptor frame_type_descriptor{};
    frame_type_descriptor.read_only = true;
    frame_type_descriptor.description = "Frame type";
    this->declare_parameter<std::string>("param_frame_type", "no name", frame_type_descriptor);

    rcl_interfaces::msg::ParameterDescriptor path_of_the_config_file_descriptor{};
    path_of_the_config_file_descriptor.read_only = true;
    path_of_the_config_file_descriptor.description = "Path of the configuaration files";
    this->declare_parameter<std::string>(
      "param_config_file_name_of_tof_sdk", "no name", path_of_the_config_file_descriptor);

    rcl_interfaces::msg::ParameterDescriptor discard_distance_threshold_mtr_descriptor{};
    discard_distance_threshold_mtr_descriptor.read_only = true;
    discard_distance_threshold_mtr_descriptor.description =
      "Discard the depth values more than the configured threshold value";
    this->declare_parameter<float>(
      "param_discard_distance_threshold_mtr", 1.5f, discard_distance_threshold_mtr_descriptor);

    rcl_interfaces::msg::ParameterDescriptor enable_compression_op_image_topics_descriptor{};
    enable_compression_op_image_topics_descriptor.read_only = true;
    enable_compression_op_image_topics_descriptor.description =
      "Enable compression on depth, IR, floor mask images";
    this->declare_parameter<bool>(
      "param_enable_compression_op_image_topics", false,
      enable_compression_op_image_topics_descriptor);

    //Run time editable parameters
    rcl_interfaces::msg::ParameterDescriptor enable_ransac_floor_detection_descriptor{};
    enable_ransac_floor_detection_descriptor.description = "Enable RANSAC Floor Detection";
    this->declare_parameter<bool>(
      "param_enable_ransac_floor_detection", true, enable_ransac_floor_detection_descriptor);

    rcl_interfaces::msg::ParameterDescriptor ransac_distance_threshold_mtr_descriptor{};
    rcl_interfaces::msg::FloatingPointRange ransac_distance_threshold_mtr_range;
    ransac_distance_threshold_mtr_range.set__from_value(0.01).set__to_value(0.05);
    ransac_distance_threshold_mtr_descriptor.floating_point_range = {
      ransac_distance_threshold_mtr_range};
    ransac_distance_threshold_mtr_descriptor.description =
      "RANSAC Distance(mtr) for Inlier Selection";
    this->declare_parameter<float>(
      "param_ransac_distance_threshold_mtr", 0.025f, ransac_distance_threshold_mtr_descriptor);

    rcl_interfaces::msg::ParameterDescriptor ransac_max_iterations_descriptor{};
    rcl_interfaces::msg::IntegerRange ransac_max_iterations_range;
    ransac_max_iterations_range.set__from_value(1).set__to_value(500);
    ransac_max_iterations_descriptor.integer_range = {ransac_max_iterations_range};
    ransac_max_iterations_descriptor.description = "Maximum number of RANSAC iterations allowed";
    this->declare_parameter<int>(
      "param_ransac_max_iterations", 10, ransac_max_iterations_descriptor);

    rcl_interfaces::msg::ParameterDescriptor enable_fallback_floor_detection_descriptor{};
    enable_fallback_floor_detection_descriptor.description =
      "Enable Fallback Floor Detection if RANSAC fails";
    this->declare_parameter<bool>(
      "param_enable_fallback_floor_detection", true, enable_fallback_floor_detection_descriptor);

    rcl_interfaces::msg::ParameterDescriptor fallback_floor_height_offset_mtr_descriptor{};
    rcl_interfaces::msg::FloatingPointRange fallback_floor_height_offset_mtr_range;
    fallback_floor_height_offset_mtr_range.set__from_value(0.01).set__to_value(0.15);
    fallback_floor_height_offset_mtr_descriptor.floating_point_range = {
      fallback_floor_height_offset_mtr_range};
    fallback_floor_height_offset_mtr_descriptor.description =
      "Floor height offset(mtr) for Fallback Floor Detector";
    this->declare_parameter<float>(
      "param_fallback_floor_height_offset_mtr", 0.1f, fallback_floor_height_offset_mtr_descriptor);

    rcl_interfaces::msg::ParameterDescriptor ab_threshold_descriptor{};
    rcl_interfaces::msg::IntegerRange ab_threshold_range;
    ab_threshold_range.set__from_value(1).set__to_value(255);
    ab_threshold_descriptor.integer_range = {ab_threshold_range};
    ab_threshold_descriptor.description = "Set ab threshold value to device";
    this->declare_parameter<int>("param_ab_threshold", 10, ab_threshold_descriptor);

    rcl_interfaces::msg::ParameterDescriptor confidence_threshold_descriptor{};
    rcl_interfaces::msg::IntegerRange confidence_threshold_range;
    confidence_threshold_range.set__from_value(1).set__to_value(255);
    confidence_threshold_descriptor.integer_range = {confidence_threshold_range};
    confidence_threshold_descriptor.description = "Set confidence threshold value to device";
    this->declare_parameter<int>("param_confidence_threshold", 10, confidence_threshold_descriptor);

    // Get Parameters
    camera_link_ = this->get_parameter("param_camera_link").as_string();
    optical_camera_link_ = this->get_parameter("param_optical_camera_link").as_string();
    input_sensor_mode_ = this->get_parameter("param_input_sensor_mode").as_int();
    output_sensor_mode_ = this->get_parameter("param_output_sensor_mode").as_int();
    input_file_name_or_ros_topic_prefix_name_ =
      this->get_parameter("param_input_file_name_or_ros_topic_prefix_name").as_string();

    std::string frame_type = this->get_parameter("param_frame_type").as_string();
    ab_threshold_ = this->get_parameter("param_ab_threshold").as_int();
    confidence_threshold_ = this->get_parameter("param_confidence_threshold").as_int();
    std::string config_file_name_of_tof_sdk =
      this->get_parameter("param_config_file_name_of_tof_sdk").as_string();

    enable_ransac_floor_detection_ =
      this->get_parameter("param_enable_ransac_floor_detection").as_bool();
    ransac_distance_threshold_mtr_ =
      (float)this->get_parameter("param_ransac_distance_threshold_mtr").as_double();
    ransac_max_iterations_ = this->get_parameter("param_ransac_max_iterations").as_int();
    discard_distance_threshold_mtr_ =
      (float)this->get_parameter("param_discard_distance_threshold_mtr").as_double();

    enable_fallback_floor_detection_ =
      this->get_parameter("param_enable_fallback_floor_detection").as_bool();
    fallback_floor_height_offset_mtr_ =
      (float)this->get_parameter("param_fallback_floor_height_offset_mtr").as_double();

    enable_compression_op_image_topics_ =
      this->get_parameter("param_enable_compression_op_image_topics").as_bool();

    //Do not modify this
    int input_image_width = 1024;
    int input_image_height = 1024;

    // Initially setting dynamic reconfigure values to same as launch file
    initSettingsForDynamicReconfigure();

    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&ADI3DToFFloorDetector::parametersCallback, this, std::placeholders::_1));

    frame_number_ = 0;
    timer_ = this->create_wall_timer(25ms, std::bind(&ADI3DToFFloorDetector::timerCallback, this));

    // Get input sensor module
    input_sensor_ = InputSensorFactory::getInputSensor(input_sensor_mode_);

    // Open the sensor
    input_sensor_->openSensor(
      input_file_name_or_ros_topic_prefix_name_, input_image_width, input_image_height,
      processing_scale_, config_file_name_of_tof_sdk);
    if (!input_sensor_->isOpened()) {
      RCLCPP_ERROR(
        this->get_logger(), "Could not open the sensor %s",
        input_file_name_or_ros_topic_prefix_name_.c_str());
      rclcpp::shutdown();
    }

    // Configure the sensor
    input_sensor_->configureSensor(frame_type);
    image_width_ = input_sensor_->getFrameWidth();
    image_height_ = input_sensor_->getFrameHeight();

    // Buffer allocation
    floor_mask_ = new unsigned char[image_width_ * image_height_];
    compressed_floor_pixels_ = (image_width_ * image_height_) / 8;
    if (enable_compression_op_image_topics_) {
      compressed_floor_mask_ = new unsigned char[compressed_floor_pixels_];
    }

    // Create TF listerner instance
    camera_map_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    camera_map_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*camera_map_tf_buffer_);

    optical_map_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    optical_map_tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*optical_map_tf_buffer_);

    // Get output sensor module
    output_sensor_ = OutputSensorFactory::getOutputSensor(output_sensor_mode_);
    if (output_sensor_ != nullptr) {
      output_sensor_->open(
        input_file_name_or_ros_topic_prefix_name_, image_width_, image_height_, EnableAllOutputs);
    }

    // Get intrinsics and extrinsics
    input_sensor_->getIntrinsics(&depth_intrinsics_);
    input_sensor_->getExtrinsics(&depth_extrinsics_);

    // Get camera link TFs
    getCameraLinksTF();

    // Object creation
    image_proc_utils_ = new ImageProcUtils(&depth_intrinsics_, image_width_, image_height_);

    floor_plane_detection_ = new FloorPlaneDetection(
      image_width_, image_height_, ransac_distance_threshold_mtr_, ransac_max_iterations_,
      discard_distance_threshold_mtr_, camera_height_mtr_);

    // Publishers
    if (enable_compression_op_image_topics_ == true) {
      compressed_depth_image_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        "depth_image/compressedDepth", 10);
      compressed_ir_image_publisher_ =
        this->create_publisher<sensor_msgs::msg::CompressedImage>("ir_image/compressedDepth", 10);
      compressed_floor_mask_publisher_ =
        this->create_publisher<sensor_msgs::msg::CompressedImage>("compressed_floor_mask", 10);
    } else {
      depth_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("depth_image", 10);
      ir_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("ir_image", 10);
      floor_mask_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("floor_mask", 10);
    }
    depth_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
    if (enable_pointcloud_publisher_) {
      xyz_image_publisher_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);
    }

    // For File-io, we do not want to miss any frame, so increasing the queue size.
    if (input_sensor_mode_ != 0) {
      max_input_queue_length_ = 100;
      max_output_queue_length_ = 100;
    }

    // setting ab thresold to pulsatrix
    input_sensor_->setABinvalidationThreshold(ab_threshold_);

    // setting confidence threshold to pulsatrix
    input_sensor_->setConfidenceThreshold(confidence_threshold_);
  }

  /**
   *
   * @brief This function runs at a given frequency and runs the algorithm
   *
   *
   */
  void timerCallback()
  {
    RCLCPP_INFO_STREAM(
      this->get_logger(), "adi_3dtof_foor_detector_node : Running loop : " << frame_number_);

    if (!runFloorDetection()) {
      rclcpp::shutdown();
    }
  }

  bool runFloorDetection();

  void processOutput();

  void processOutputAbort();

  void readInputAbort();

  void readInput();

  ADTF31xxSensorFrameInfo * floorDetectorIOThreadGetNextFrame(void);

  ADI3DToFFloorDetectorOutputInfo * floorDetectorIOThreadGetNext(void);

  void updateDynamicReconfigureVariablesInputThread();

  void updateDynamicReconfigureVariablesProcessThread();

  /**
   * @brief Destroy the ADI3DToFFloorDetector object
   *
   */
  ~ADI3DToFFloorDetector()
  {
    // Close the input sensor
    input_sensor_->closeSensor();

    delete[] floor_mask_;
    if (enable_compression_op_image_topics_) {
      delete[] compressed_floor_mask_;
    }

    delete image_proc_utils_;
    delete floor_plane_detection_;

    // Close outputs
    if (output_sensor_ != nullptr) {
      output_sensor_->close();
    }
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  IInputSensor * input_sensor_;
  OOutputSensor * output_sensor_;
  ImageProcUtils * image_proc_utils_;
  FloorPlaneDetection * floor_plane_detection_;
  int image_width_;
  int image_height_;
  int frame_number_;
  int input_sensor_mode_;
  int output_sensor_mode_;
  std::string camera_link_;
  std::string optical_camera_link_;
  bool enable_compression_op_image_topics_ = false;
  bool enable_pointcloud_publisher_ = false;
  int ab_threshold_ = 10;
  int confidence_threshold_ = 10;

  std::unique_ptr<tf2_ros::Buffer> camera_map_tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> camera_map_tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> optical_map_tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> optical_map_tf_listener_{nullptr};

  std::string input_file_name_or_ros_topic_prefix_name_;
  std::string output_file_name_;
  sensor_msgs::msg::CameraInfo cam_info_msg_;
  unsigned short * depth_frame_ = nullptr;
  unsigned short * ir_frame_ = nullptr;
  short * xyz_frame_ = nullptr;
  short * input_xyz_frame_ = nullptr;
  short * output_xyz_frame_ = nullptr;
  int compressed_depth_frame_size_ = 0;
  int compressed_ir_frame_size_ = 0;
  unsigned char * compressed_depth_frame_ = nullptr;
  unsigned char * compressed_ir_frame_ = nullptr;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ir_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr floor_mask_publisher_;

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_depth_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_ir_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_floor_mask_publisher_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr xyz_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_publisher_;

  CameraIntrinsics depth_intrinsics_;
  CameraExtrinsics depth_extrinsics_;
  CameraExtrinsics depth_extrinsics_external_;

  bool process_output_thread_abort_ = false;
  bool read_input_thread_abort_ = false;

  std::mutex output_thread_mtx_;
  std::mutex input_thread_mtx_;
  int max_input_queue_length_ = 1;
  std::queue<ADTF31xxSensorFrameInfo *> input_frames_queue_;
  int max_output_queue_length_ = 5;
  std::queue<ADI3DToFFloorDetectorOutputInfo *> output_node_queue_;
  bool error_in_frame_read_ = false;
  /**
   * @brief    This is the scale factor to scale the input image.
               The processing will happen on the scaled down image
               The topics and the output coordinates will correspond
               to the scaled image size.
   *
   */
  int processing_scale_ = 2;
  rclcpp::Time curr_frame_timestamp_ = rclcpp::Clock{}.now();
  struct DynamicReconfigureParameters
  {
    int ab_threshold;
    int confidence_threshold;
    bool enable_ransac_floor_detection;
    int ransac_max_iterations;
    float ransac_distance_threshold_mtr;
    bool enable_fallback_floor_detection;
    float fallback_floor_height_offset_mtr;
  };

  DynamicReconfigureParameters dynamic_reconf_params_;

  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  /*Floor Detection Variables*/
  /**
   * @brief Variable to choose which floor detection algorithm to run
   * 1. RANSAC based (default)
   * 2. Threshold Y based
   *
   */
  bool enable_ransac_floor_detection_;

  /**
   * @brief Enable Fallback floor detection algorithm(Threshold Y based) if RANSAC fails
   *
   */
  bool enable_fallback_floor_detection_;

  /**
   * @brief Result status of RANSAC floor detection
   * If the status is true then RANSAC result can be considered, otherwise, run the Threshold Y based floor detection.
   *
   */
  bool ransac_floor_detection_status_ = false;

  /**
   * @brief Rotation around X axis in radian
   *
   */
  float camera_roll_rad_ = 0.0f;

  /**
   * @brief Rotation around Y axis in radian
   *
   */
  float camera_pitch_rad_ = 0.0f;

  /**
   * @brief Rotation around Z axis in radian
   *
   */
  float camera_yaw_rad_ = 0.0f;

  /**
   * @brief Flag for camera rotation
   *
   */
  bool camera_tilted_ = false;

  /**
   * @brief distance (25mm) which determines how close the point must be to the RANSAC plane in order to be selected as
   * inlier.
   *
   */
  float ransac_distance_threshold_mtr_ = 0.025f;

  /**
   * @brief Maximum number of RANSAC iterations (10) which is allowed
   *
   */
  int ransac_max_iterations_ = 10;

  /**
   * @brief Camera height, default is 0.1524m (6 inches)
   *
   */
  float camera_height_mtr_ = 0.1524f;

  /**
   * @brief Threshold to filter the point cloud based on depth (Z)
   * Select the points with depth value lesser than 1.5m from the sensor
   *
   */
  float discard_distance_threshold_mtr_ = 1.5f;

  /**
   * @brief Points which are lower than floor
   *
   */
  int noise_count_ = 0;

  /**
   * @brief Number of iterations RANSAC has taken for floor detection
   *
   */
  int ransac_iterations_ = 0;

  /**
   * @brief Time (ms) RANSAC has taken for floor detection
   *
   */
  float ransac_time_ms_ = 0.0f;

  /**
   * @brief Floor height with offset for Threshold Y based floor detection.
   *
   */
  float fallback_floor_height_offset_mtr_;

  /**
   * @brief Floor distnace threhsold wrt sensor
   *
   */
  float floor_distance_threshold_mm_;

  /**
   * @brief Floor mask
   *
   */
  unsigned char * floor_mask_ = nullptr;

  /**
   * @brief Compressed floor mask
   *
   */
  unsigned char * compressed_floor_mask_ = nullptr;

  /**
   * @brief Compressed floor pixels count
   *
   */
  int compressed_floor_pixels_ = 0;

  /**Function Declarations*/

  void publishImageAndCameraInfo(
    unsigned short * depth_frame, unsigned short * ir_frame, unsigned char * floor_mask_frame,
    short * xyz_frame);

  void publishImageAndCameraInfo(
    unsigned char * compressed_depth_frame, int compressed_depth_frame_size,
    unsigned char * compressed_ir_frame, int compressed_ir_frame_size,
    unsigned char * compressed_floor_mask_frame, int compressed_floor_mask_frame_size,
    short * xyz_frame);

  void fillAndPublishCameraInfo(
    const std::string & frame_id,
    const rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher);

  void publishImageAsRosMsg(
    const cv::Mat & img, const std::string & encoding_type, const std::string & frame_id,
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher);

  void publishCompressedImageAsRosMsg(
    unsigned char * compressed_img, int compressed_img_size, const std::string & encoding_type,
    const std::string & frame_id,
    const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher);

  void publishPointCloud(short * xyz_frame);

  cv::Mat getFloorDetectionOutput(
    unsigned short * depth_frame_with_floor, unsigned short * ir_frame,
    unsigned char * floor_mask_8bit);

  void initSettingsForDynamicReconfigure();

  void getCameraLinksTF();

  bool processOutputDone();

  void processOutputStart();

  void floorDetectorIOThreadPushOutputNode(ADI3DToFFloorDetectorOutputInfo * new_output_node);

  void rotatePointCloud(
    short * input_point_cloud, short * rotated_point_cloud, CameraExtrinsics * camera_extrinsics,
    int image_width, int image_height);

  /**
   * @brief new values from dynamic reconfigure are copied to a struture variable here, actual update to individual
   * parameters happens in updateDynamicReconfigureVariablesInputThread and updateDynamicReconfigureVariablesProcessThread
   * functions.
  */
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    // Copy the parameters vector to a local variable.
    for (const auto & param : parameters) {
      if (param.get_name() == "param_enable_ransac_floor_detection") {
        if (dynamic_reconf_params_.enable_ransac_floor_detection != param.as_bool()) {
          dynamic_reconf_params_.enable_ransac_floor_detection = param.as_bool();
          RCLCPP_INFO(
            this->get_logger(), "The value of param_enable_ransac_floor_detection is changed to %s",
            param.value_to_string().c_str());
        }
      }

      else if (param.get_name() == "param_ransac_max_iterations") {
        if (dynamic_reconf_params_.ransac_max_iterations != param.as_int()) {
          dynamic_reconf_params_.ransac_max_iterations = param.as_int();
          RCLCPP_INFO(
            this->get_logger(), "The value of param_ransac_max_iterations is changed to %s",
            param.value_to_string().c_str());
        }
      }

      else if (param.get_name() == "param_ransac_distance_threshold_mtr") {
        if (dynamic_reconf_params_.ransac_distance_threshold_mtr != (float)param.as_double()) {
          dynamic_reconf_params_.ransac_distance_threshold_mtr = (float)param.as_double();
          RCLCPP_INFO(
            this->get_logger(), "The value of param_ransac_distance_threshold_mtr is changed to %s",
            param.value_to_string().c_str());
        }
      }

      else if (param.get_name() == "param_enable_fallback_floor_detection") {
        if (dynamic_reconf_params_.enable_fallback_floor_detection != param.as_bool()) {
          dynamic_reconf_params_.enable_fallback_floor_detection = param.as_bool();
          RCLCPP_INFO(
            this->get_logger(),
            "The value of param_enable_fallback_floor_detection is changed to %s",
            param.value_to_string().c_str());
        }
      }

      else if (param.get_name() == "param_fallback_floor_height_offset_mtr") {
        if (dynamic_reconf_params_.fallback_floor_height_offset_mtr != (float)param.as_double()) {
          dynamic_reconf_params_.fallback_floor_height_offset_mtr = (float)param.as_double();
          RCLCPP_INFO(
            this->get_logger(),
            "The value of param_fallback_floor_height_offset_mtr is changed to %s",
            param.value_to_string().c_str());
        }
      }

      else if (param.get_name() == "param_ab_threshold") {
        if (dynamic_reconf_params_.ab_threshold != param.as_int()) {
          dynamic_reconf_params_.ab_threshold = param.as_int();
          RCLCPP_INFO(
            this->get_logger(), "The value of param_ab_threshold is changed to %s",
            param.value_to_string().c_str());
        }
      }

      else if (param.get_name() == "param_confidence_threshold") {
        if (dynamic_reconf_params_.confidence_threshold != param.as_int()) {
          dynamic_reconf_params_.confidence_threshold = param.as_int();
          RCLCPP_INFO(
            this->get_logger(), "The value of param_confidence_threshold is changed to %s",
            param.value_to_string().c_str());
        }
      }
    }
    return result;
  }
};

#endif
