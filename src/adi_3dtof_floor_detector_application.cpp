/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#include "adi_3dtof_floor_detector_node.h"
//#include "ros-perception/image_transport_plugins/compressed_depth_image_transport/rvl_codec.h"
#include "ros-perception/image_transport_plugins/compressed_depth_image_transport/compression_common.h"
/**
 * @brief This function publishes depth image , ir image, point-cloud and camera info.
 *
 * @param depth_frame - Pointer to the depth frame buffer
 * @param ab_frame - Pointer to the ir frame buffer
 * @param floor_mask_frame - Pointer to the Floor Mask buffer
 * @param xyz_frame - Pointer to the xyz frame buffer
 */
void ADI3DToFFloorDetector::publishImageAndCameraInfo(
  unsigned short * depth_frame, unsigned short * ab_frame, unsigned char * floor_mask_frame,
  short * xyz_frame)
{
  // Publish image as Ros message
  cv::Mat m_disp_image_depth, m_disp_image_ir, m_disp_image_floor_mask;

  // convert to 16 bit depth and IR image of CV format.
  m_disp_image_depth = cv::Mat(image_height_, image_width_, CV_16UC1, depth_frame);
  m_disp_image_ir = cv::Mat(image_height_, image_width_, CV_16UC1, ab_frame);
  m_disp_image_floor_mask = cv::Mat(image_height_, image_width_, CV_8UC1, floor_mask_frame);

  fillAndPublishCameraInfo(optical_camera_link_, depth_info_publisher_);
  // encoding type should not be mono as nodelet expects it to be in enc format
  // frame id should be the frame name not topic name
  publishImageAsRosMsg(m_disp_image_depth, "mono16", optical_camera_link_, depth_image_publisher_);
  publishImageAsRosMsg(m_disp_image_ir, "mono16", optical_camera_link_, ab_image_publisher_);
  publishImageAsRosMsg(
    m_disp_image_floor_mask, "mono8", optical_camera_link_, floor_mask_publisher_);

  if (enable_pointcloud_publisher_) {
    publishPointCloud(xyz_frame);
  }
}

/**
 * @brief This function publishes camera info and compressed version of depth, ir, floor mask images
 *
 * @param compressed_depth_frame Compressed depth image
 * @param compressed_depth_frame_size Compressed depth image size
 * @param compressed_ab_frame Compressed ir image
 * @param compressed_ab_frame_size Compressed ir image size
 * @param compressed_floor_mask_frame Compressed floor mask image
 * @param compressed_floor_mask_frame_size Compressed floor mask image size
 * @param xyz_frame point cloud buffer
 */
void ADI3DToFFloorDetector::publishImageAndCameraInfo(
  unsigned char * compressed_depth_frame, int compressed_depth_frame_size,
  unsigned char * compressed_ab_frame, int compressed_ab_frame_size,
  unsigned char * compressed_floor_mask_frame, int compressed_floor_mask_frame_size,
  short * xyz_frame)
{
  fillAndPublishCameraInfo(optical_camera_link_, depth_info_publisher_);

  publishCompressedImageAsRosMsg(
    compressed_depth_frame, compressed_depth_frame_size, "mono16", optical_camera_link_,
    compressed_depth_image_publisher_);

  publishCompressedImageAsRosMsg(
    compressed_ab_frame, compressed_ab_frame_size, "mono16", optical_camera_link_,
    compressed_ab_image_publisher_);

  publishCompressedImageAsRosMsg(
    compressed_floor_mask_frame, compressed_floor_mask_frame_size, "mono8", optical_camera_link_,
    compressed_floor_mask_publisher_);

  if (enable_pointcloud_publisher_) {
    publishPointCloud(xyz_frame);
  }
}

/**
 * @brief This image fills and publishes the camera information
 *
 * @param frame_id  frame_id of camera_info
 * @param publisher This is Ros publisher
 */
void ADI3DToFFloorDetector::fillAndPublishCameraInfo(
  const std::string & frame_id,
  const rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher)
{
  cam_info_msg_.header.stamp = curr_frame_timestamp_;
  cam_info_msg_.header.frame_id = frame_id;

  cam_info_msg_.width = image_width_;
  cam_info_msg_.height = image_height_;

  cam_info_msg_.distortion_model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;

  cam_info_msg_.k.fill(0.0f);
  cam_info_msg_.k[0] = depth_intrinsics_.camera_matrix[0];
  cam_info_msg_.k[2] = depth_intrinsics_.camera_matrix[2];
  cam_info_msg_.k[4] = depth_intrinsics_.camera_matrix[4];
  cam_info_msg_.k[5] = depth_intrinsics_.camera_matrix[5];
  cam_info_msg_.k[8] = 1.0f;

  cam_info_msg_.p.fill(0.0);
  cam_info_msg_.p[0] = depth_intrinsics_.camera_matrix[0];
  cam_info_msg_.p[2] = depth_intrinsics_.camera_matrix[2];
  cam_info_msg_.p[3] = depth_extrinsics_.translation_matrix[0];
  cam_info_msg_.p[5] = depth_intrinsics_.camera_matrix[4];
  cam_info_msg_.p[6] = depth_intrinsics_.camera_matrix[5];
  cam_info_msg_.p[7] = depth_extrinsics_.translation_matrix[1];
  cam_info_msg_.p[10] = 1.0f;
  cam_info_msg_.p[11] = depth_extrinsics_.translation_matrix[2];

  cam_info_msg_.d.resize(0);
  for (float distortion_coeff : depth_intrinsics_.distortion_coeffs) {
    cam_info_msg_.d.push_back(distortion_coeff);
  }

  cam_info_msg_.r.fill(0.0f);
  for (int i = 0; i < 9; i++) {
    cam_info_msg_.r[i] = depth_extrinsics_.rotation_matrix[i];
  }

  cam_info_msg_.binning_x = 0;
  cam_info_msg_.binning_y = 0;
  cam_info_msg_.roi.do_rectify = false;
  cam_info_msg_.roi.height = 0;
  cam_info_msg_.roi.width = 0;
  cam_info_msg_.roi.x_offset = 0;
  cam_info_msg_.roi.y_offset = 0;

  publisher->publish(cam_info_msg_);
}

/**
 * @brief This function publishes a image(of cv::Mat() type) as Ros message.
 *
 * @param img Input image
 * @param encoding_type number of bits used to represent one pixel of image.
 * @param frame_id frame id of image
 * @param publisher ROS publisher handle
 * @param enable_image_compression Image compression Flag
 */
void ADI3DToFFloorDetector::publishImageAsRosMsg(
  const cv::Mat & img, const std::string & encoding_type, const std::string & frame_id,
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher)
{
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  cv_ptr->encoding = encoding_type;
  cv_ptr->header.stamp = curr_frame_timestamp_;
  cv_ptr->header.frame_id = std::move(frame_id);
  cv_ptr->image = std::move(img);

  publisher->publish(*cv_ptr->toImageMsg());
}

/**
 * @brief Publishes the compressed image as ROS message
 *
 * @param compressed_img input image
 * @param compressed_img_size input image size
 * @param encoding_type encodding type
 * @param frame_id frame id
 * @param publisher output image publisher
 */
void ADI3DToFFloorDetector::publishCompressedImageAsRosMsg(
  unsigned char * compressed_img, int compressed_img_size, const std::string & encoding_type,
  const std::string & frame_id,
  const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher)
{
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  sensor_msgs::msg::CompressedImage::SharedPtr compressed_payload_ptr(
    new sensor_msgs::msg::CompressedImage());

  compressed_payload_ptr->format = encoding_type + ";compressedDepth rvl";
  compressed_payload_ptr->header.stamp = curr_frame_timestamp_;
  compressed_payload_ptr->header.frame_id = std::move(frame_id);
  compressed_payload_ptr->data.resize(
    compressed_img_size + 8 + sizeof(compressed_depth_image_transport::ConfigHeader));

  // Image compression configuration
  compressed_depth_image_transport::ConfigHeader compression_config{};
  compression_config.format = compressed_depth_image_transport::INV_DEPTH;

  float depth_z0 = 0;
  float depth_max = 0;

  // Inverse depth quantization parameters
  float depth_quant_a = depth_z0 * (depth_z0 + 1.0f);
  float depth_quant_b = 1.0f - depth_quant_a / depth_max;

  // Add coding parameters to header
  compression_config.depthParam[0] = depth_quant_a;
  compression_config.depthParam[1] = depth_quant_b;

  memcpy(
    &compressed_payload_ptr->data[0], &compression_config,
    sizeof(compressed_depth_image_transport::ConfigHeader));
  memcpy(
    &compressed_payload_ptr->data[0] + sizeof(compressed_depth_image_transport::ConfigHeader),
    &image_width_, sizeof(int));
  memcpy(
    &compressed_payload_ptr->data[4] + sizeof(compressed_depth_image_transport::ConfigHeader),
    &image_height_, sizeof(int));
  memcpy(
    &compressed_payload_ptr->data[8] + sizeof(compressed_depth_image_transport::ConfigHeader),
    compressed_img, compressed_img_size);

  publisher->publish(*compressed_payload_ptr);
}

/**
 * @brief This function publishes the point cloud
 *
 * @param xyz_frame Buffer containing the xyz values in interleaved format
 *
 * Note: Assumes that cam_info_msg_ is already populated
 */
void ADI3DToFFloorDetector::publishPointCloud(short * xyz_frame)
{
  sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg(new sensor_msgs::msg::PointCloud2);

  pointcloud_msg->header.stamp = curr_frame_timestamp_;
  pointcloud_msg->header.frame_id = optical_camera_link_;
  pointcloud_msg->width = image_width_;
  pointcloud_msg->height = image_height_;
  pointcloud_msg->is_dense = false;
  pointcloud_msg->is_bigendian = false;

  // XYZ data from sensor.
  // This data is in 16 bpp format.
  short * xyz_sensor_buf;
  xyz_sensor_buf = xyz_frame;
  sensor_msgs::PointCloud2Modifier pcd_modifier(*pointcloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

  sensor_msgs::PointCloud2Iterator<float> iter_x(*pointcloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*pointcloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*pointcloud_msg, "z");
  for (int i = 0; i < image_height_; i++) {
    for (int j = 0; j < image_width_; j++) {
      *iter_x = (float)(*xyz_sensor_buf++) / 1000.0f;
      *iter_y = (float)(*xyz_sensor_buf++) / 1000.0f;
      *iter_z = (float)(*xyz_sensor_buf++) / 1000.0f;
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
  }

  // Publisher
  xyz_image_publisher_->publish(*pointcloud_msg);
}

/**
 * @brief Gives output frame for floor detection video
 *
 * @param depth_frame_with_floor Original depth frame
 * @param ab_frame Original IR frame
 * @param floor_mask_8bit Floor Mask
 * @return cv::Mat Final frame for Output Video
 */
cv::Mat ADI3DToFFloorDetector::getFloorDetectionOutput(
  unsigned short * depth_frame_with_floor, unsigned short * ab_frame,
  unsigned char * floor_mask_8bit)
{
  // Gamma Correction for 16bit IR image
  for (int i = 0; i < (image_width_ * image_height_); i++) {
    float read = (float)ab_frame[i];
    float out_val = (float)(256.0f * log(read)) / log(2048.0f);
    ab_frame[i] = (uint16_t)out_val;
  }

  // Get 8bit IR image
  int max = 0, min = 65535;
  for (int i = 0; i < (image_width_ * image_height_); i++) {
    if (ab_frame[i] > max) {
      max = ab_frame[i];
    }
    if (ab_frame[i] < min) {
      min = ab_frame[i];
    }
  }
  cv::Mat ab_image_8bit = cv::Mat::zeros(image_height_, image_width_, CV_8UC1);
  for (int i = 0; i < (image_width_ * image_height_); i++) {
    ab_image_8bit.data[i] = (ab_frame[i] - min) / (float)(max - min) * 255;
  }

  // Get rgb 8 bit ir image
  cv::Mat ab_image_8bit_rgb = cv::Mat::zeros(image_height_, image_width_, CV_8UC3);
  cv::cvtColor(ab_image_8bit, ab_image_8bit_rgb, cv::COLOR_GRAY2BGR);

  // convert 16bit depth image with floor to 8bit
  cv::Mat depth_image_16bit =
    cv::Mat(image_height_, image_width_, CV_16UC1, depth_frame_with_floor);
  cv::Mat depth_image_8bit = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC1);
  unsigned short max_element = 8192;
  float scale_factor = 255.0f / max_element;
  depth_image_16bit.convertTo(depth_image_8bit, CV_8UC1, scale_factor, 0);

  // convert 8bit depth image to rgb
  cv::Mat depth_image_8bit_rgb = cv::Mat::zeros(image_height_, image_width_, CV_8UC3);
  cv::cvtColor(depth_image_8bit, depth_image_8bit_rgb, cv::COLOR_GRAY2BGR);

  // Concatenation of input depth and IR images
  cv::Mat ab_depth_op_image = cv::Mat::zeros(cv::Size(image_width_ * 2, image_height_), CV_8UC3);
  cv::hconcat(ab_image_8bit_rgb, depth_image_8bit_rgb, ab_depth_op_image);

  // Get floor marked output
  cv::Mat floor_mask_8bit_image = cv::Mat(image_height_, image_width_, CV_8UC1, floor_mask_8bit);

  // Only if floor exists in the depth image
  int non_zero_pixels = cv::countNonZero(floor_mask_8bit_image);
  if (non_zero_pixels > 0) {
    cv::Mat channels[3];
    cv::split(depth_image_8bit_rgb, channels);
    cv::bitwise_or(channels[1], floor_mask_8bit_image, channels[1]);
    cv::merge(channels, 3, depth_image_8bit_rgb);
  }

  // Concatenate floor detection output
  cv::Mat final_out_image = cv::Mat::zeros(cv::Size(image_width_ * 3, image_height_), CV_8UC3);
  cv::hconcat(ab_depth_op_image, depth_image_8bit_rgb, final_out_image);

  return final_out_image;
}

/**
 * @brief Overwriting the parameters read from Dynamic Reconfigure only in init time
 * This is done to make sure that launch file has higher priority than dynamic reconfigure
 * in init time.
 *
 */
void ADI3DToFFloorDetector::initSettingsForDynamicReconfigure()
{
  dynamic_reconf_params_.enable_ransac_floor_detection = enable_ransac_floor_detection_;
  dynamic_reconf_params_.ransac_max_iterations = ransac_max_iterations_;
  dynamic_reconf_params_.ransac_distance_threshold_mtr = ransac_distance_threshold_mtr_;
  dynamic_reconf_params_.enable_fallback_floor_detection = enable_fallback_floor_detection_;
  dynamic_reconf_params_.fallback_floor_height_offset_mtr = fallback_floor_height_offset_mtr_;
  dynamic_reconf_params_.ab_threshold = ab_threshold_;
  dynamic_reconf_params_.confidence_threshold = confidence_threshold_;
}
