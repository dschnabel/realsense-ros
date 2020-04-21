/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Chad Rockey
 */

#include <depthimage_to_laserscan/DepthImageToLaserScan.h>

using namespace depthimage_to_laserscan;

DepthImageToLaserScan::DepthImageToLaserScan()
  : scan_time_(1./30.)
  , range_min_(0.45)
  , range_max_(10.0)
  , scan_height_(1)
  , scan_tilt_(0)
{
    /*
     * Here we calculate the floor matrix with polynomial regression equations. The coefficients
     * were acquired using acutal measured distances from the camera to the floor.
     */

    int mostLeftCol = 1, leftCol = 12, centerCol = 211, rightCol = 410, mostRightCol = FLOOR_MATRIX_COLS;
    int rows = FLOOR_MATRIX_ROWS;

    // quadradic regression for distance to floor (3 coefficients)
    double co1Left = 0.465139, co1Center = 0.388712, co1Right = 0.454160;
    double co1CenterLeftSteps = abs(co1Left - co1Center) / (double)(centerCol - leftCol);
    double co1CenterRightSteps = abs(co1Right - co1Center) / (double)(rightCol - centerCol);

    double co2Left = 0.002115389833247224, co2Center = 0.0017127089022463305, co2Right = 0.0012068192857903144;
    double co2CenterLeftSteps = abs(co2Left - co2Center) / (double)(centerCol - leftCol);
    double co2CenterRightSteps = abs(co2Right - co2Center) / (double)(rightCol - centerCol);

    double co3Left = 0.0001507743215119495, co3Center = 0.00012619343106256133, co3Right = 0.00018707445822834342;
    double co3CenterLeftSteps = abs(co3Left - co3Center) / (double)(centerCol - leftCol);
    double co3CenterRightSteps = abs(co3Right - co3Center) / (double)(rightCol - centerCol);

    // linear regression for max deviation to optimal value (2 coefficients)
    double deCo1Left = 0.0033566764705882353, deCo1Center = 0.006114178571428572, deCo1Right = 0.009170321428571429;
    double deCo1CenterLeftSteps = abs(deCo1Left - deCo1Center) / (double)(centerCol - leftCol);
    double deCo1CenterRightSteps = abs(deCo1Right - deCo1Center) / (double)(rightCol - centerCol);

    double deCo2Left = 0.0007411470588235294, deCo2Center = 0.00026210714285714287, deCo2Right = 0.0006376071428571429;
    double deCo2CenterLeftSteps = abs(deCo2Left - deCo2Center) / (double)(centerCol - leftCol);
    double deCo2CenterRightSteps = abs(deCo2Right - deCo2Center) / (double)(rightCol - centerCol);

    double co1 = co1Center;
    double co2 = co2Center;
    double co3 = co3Center;
    double deCo1 = deCo1Center;
    double deCo2 = deCo2Center;
    for (int x = centerCol; x >= mostLeftCol; x--) {
        for (int y = 1; y <= rows; y++) {
            floor_matrix_[x-1][y-1] = std::make_pair(co1 + co2 * y + co3 * pow(y,2), deCo1 + deCo2 * y);
        }
        co1 += co1CenterLeftSteps;
        co2 += co2CenterLeftSteps;
        co3 += co3CenterLeftSteps;
        deCo1 -= deCo1CenterLeftSteps; // left lower than center, use minus
        deCo2 += deCo2CenterLeftSteps;
    }

    co1 = co1Center;
    co2 = co2Center;
    co3 = co3Center;
    deCo1 = deCo1Center;
    deCo2 = deCo2Center;
    for (int x = centerCol; x <= mostRightCol; x++) {
        for (int y = 1; y <= rows; y++) {
            floor_matrix_[x-1][y-1] = std::make_pair(co1 + co2 * y + co3 * pow(y,2), deCo1 + deCo2 * y);
        }
        co1 += co1CenterRightSteps;
        co2 -= co2CenterRightSteps; // right lower than center, use minus
        co3 += co3CenterRightSteps;
        deCo1 += deCo1CenterRightSteps;
        deCo2 += deCo2CenterRightSteps;
    }

}

DepthImageToLaserScan::~DepthImageToLaserScan(){
}

double DepthImageToLaserScan::magnitude_of_ray(const cv::Point3d& ray) const{
  return sqrt(pow(ray.x, 2.0) + pow(ray.y, 2.0) + pow(ray.z, 2.0));
}

double DepthImageToLaserScan::angle_between_rays(const cv::Point3d& ray1, const cv::Point3d& ray2) const{
  const double dot_product = ray1.x*ray2.x + ray1.y*ray2.y + ray1.z*ray2.z;
  const double magnitude1 = magnitude_of_ray(ray1);
  const double magnitude2 = magnitude_of_ray(ray2);;
  return acos(dot_product / (magnitude1 * magnitude2));
}

bool DepthImageToLaserScan::use_point(const float new_value, const float old_value, const float range_min, const float range_max) const{
  // Check for NaNs and Infs, a real number within our limits is more desirable than these.
  const bool new_finite = std::isfinite(new_value);
  const bool old_finite = std::isfinite(old_value);

  // Infs are preferable over NaNs (more information)
  if(!new_finite && !old_finite){ // Both are not NaN or Inf.
    if(!std::isnan(new_value)){ // new is not NaN, so use it's +-Inf value.
      return true;
    }
    return false; // Do not replace old_value
  }

  // If not in range, don't bother
  const bool range_check = range_min <= new_value && new_value <= range_max;
  if(!range_check){
    return false;
  }

  if(!old_finite){ // New value is in range and finite, use it.
    return true;
  }

  // Finally, if they are both numerical and new_value is closer than old_value, use new_value.
  const bool shorter_check = new_value < old_value;
  return shorter_check;
}

template<typename T>
void DepthImageToLaserScan::convert(const cv::Mat& image, const image_geometry::PinholeCameraModel& cam_model,
    const sensor_msgs::LaserScanPtr& scan_msg) const{
  // Use correct principal point from calibration
  const float center_x = cam_model.cx();
  const int floor_y = image.rows;

  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  const double unit_scaling = depthimage_to_laserscan::DepthTraits<T>::toMeters( T(1) );
  const float constant_x = unit_scaling / cam_model.fx();

  const T* depth_row = reinterpret_cast<const T*>(&image.data[0]);
  const int row_step = (image.cols * image.elemSize()) / sizeof(T);

  const int offset = floor_y - scan_height_;
  depth_row += offset*row_step;

  const T* depth_row_cpy = depth_row;
  int index_old = -1;

  for (int u = 0; u < (int)image.cols; ++u) { // Loop over each pixel in row
    const double th = -atan2((double)(u - center_x) * constant_x, unit_scaling); // Atan2(x, z), but depth divides out
    const int index = (th - scan_msg->angle_min) / scan_msg->angle_increment;

    if (index == index_old) continue;
    index_old = index;

    depth_row = depth_row_cpy;

    for(int v = offset; v < offset+scan_height_; ++v, depth_row += row_step){
      const T depth = depth_row[u];
      double r = depth; // Assign to pass through NaNs and Infs

      if (depthimage_to_laserscan::DepthTraits<T>::valid(depth)){ // Not NaN or Inf
        // Calculate in XYZ
        double x = (u - center_x) * depth * constant_x;
        double z = depthimage_to_laserscan::DepthTraits<T>::toMeters(depth);

        // Calculate actual distance
        r = hypot(x, z);
      }

      // Determine if this point should be used.
      if(use_point(r, scan_msg->ranges[index], scan_msg->range_min, scan_msg->range_max)){
        scan_msg->ranges[index] = r;
      }
    }
  }
}

//#include <numeric>
//void my_stat_function(const cv::Mat& image, const image_geometry::PinholeCameraModel& cam_model,
//        const sensor_msgs::LaserScanPtr& scan_msg) {
//
//    static int counter = 0;
//    static std::map<int,std::vector<double>> a;
//
//    // Use correct principal point from calibration
//    const float center_x = cam_model.cx();
//    const int floor_y = image.rows;
//    int y_height = 50;
//    int my_col = 12;
//
//    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
//    const double unit_scaling = depthimage_to_laserscan::DepthTraits<uint16_t>::toMeters( uint16_t(1) );
//    const float constant_x = unit_scaling / cam_model.fx();
//
//    const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(&image.data[0]);
//    const int row_step = (image.cols * image.elemSize()) / sizeof(uint16_t);
//
//    const int offset = floor_y - y_height;
//    depth_row += offset*row_step;
//
//    const uint16_t* depth_row_cpy = depth_row;
//
//    int index_old = -1;
//
//    for (int u = my_col; u < my_col+1; ++u) {
//        const double th = -atan2((double)(u - center_x) * constant_x, unit_scaling);
//        const int index = (th - scan_msg->angle_min) / scan_msg->angle_increment;
//
//        if (index == index_old) continue;
//        index_old = index;
//        depth_row = depth_row_cpy;
//
////        printf("%d:",index);
//        for(int v = offset; v < offset+y_height; ++v, depth_row += row_step){
//            const uint16_t depth = depth_row[u];
//            double r = depth; // Assign to pass through NaNs and Infs
//
//            if (depthimage_to_laserscan::DepthTraits<uint16_t>::valid(depth)){ // Not NaN or Inf
//                // Calculate in XYZ
//                double x = (u - center_x) * depth * constant_x;
//                double z = depthimage_to_laserscan::DepthTraits<uint16_t>::toMeters(depth);
//
//                // Calculate actual distance
//                r = hypot(x, z);
//            }
//
//            if (r > 0) {
//                int i = offset+y_height-v;
//                a[i].push_back(r);
//            }
//        }
//    }
//
//    if (counter++ == 300) {
//        for (auto &b : a) {
//            std::vector<double> &v = b.second;
//            printf("%d %f,", b.first, std::accumulate( v.begin(), v.end(), 0.0) / v.size());
//        }
//        printf("\n===\n");
//        a.clear();
//        counter = 0;
//    }
//}

sensor_msgs::LaserScanPtr DepthImageToLaserScan::convert_msg(const cv::Mat& image, std::string& encoding,
        const sensor_msgs::CameraInfo& info_msg){
  // Set camera model
  cam_model_.fromCameraInfo(info_msg);

  // Calculate angle_min and angle_max by measuring angles between the left ray, right ray, and optical center ray
  cv::Point2d raw_pixel_left(0, cam_model_.cy());
  cv::Point2d rect_pixel_left = cam_model_.rectifyPoint(raw_pixel_left);
  cv::Point3d left_ray = cam_model_.projectPixelTo3dRay(rect_pixel_left);

  cv::Point2d raw_pixel_right(image.cols-1, cam_model_.cy());
  cv::Point2d rect_pixel_right = cam_model_.rectifyPoint(raw_pixel_right);
  cv::Point3d right_ray = cam_model_.projectPixelTo3dRay(rect_pixel_right);

  cv::Point2d raw_pixel_center(cam_model_.cx(), cam_model_.cy());
  cv::Point2d rect_pixel_center = cam_model_.rectifyPoint(raw_pixel_center);
  cv::Point3d center_ray = cam_model_.projectPixelTo3dRay(rect_pixel_center);

  const double angle_max = angle_between_rays(left_ray, center_ray);
  const double angle_min = -angle_between_rays(center_ray, right_ray); // Negative because the laserscan message expects an opposite rotation of that from the depth image

  // Fill in laserscan message
  sensor_msgs::LaserScanPtr scan_msg(new sensor_msgs::LaserScan());
  scan_msg->header = info_msg.header;
  if(output_frame_id_.length() > 0){
    scan_msg->header.frame_id = output_frame_id_;
  }
  scan_msg->angle_min = angle_min;
  scan_msg->angle_max = angle_max;
  scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (image.cols - 1);
  scan_msg->time_increment = 0.0;
  scan_msg->scan_time = scan_time_;
  scan_msg->range_min = range_min_;
  scan_msg->range_max = range_max_;

  // Check scan_height vs image_height
  if(scan_height_/2 > cam_model_.cy() || scan_height_/2 > image.rows - cam_model_.cy()){
    std::stringstream ss;
    ss << "scan_height ( " << scan_height_ << " pixels) is too large for the image height.";
    throw std::runtime_error(ss.str());
  }

  // Calculate and fill the ranges
  const uint32_t ranges_size = image.cols;
  scan_msg->ranges.assign(ranges_size, std::numeric_limits<float>::quiet_NaN());

  if (encoding == sensor_msgs::image_encodings::TYPE_16UC1)
  {
    convert<uint16_t>(image, cam_model_, scan_msg);
  }
  else if (encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    convert<float>(image, cam_model_, scan_msg);
  }
  else
  {
    std::stringstream ss;
    ss << "Depth image has unsupported encoding: " << encoding;
    throw std::runtime_error(ss.str());
  }

//  my_stat_function(image, cam_model_, scan_msg);

  return scan_msg;
}

void DepthImageToLaserScan::set_scan_time(const float scan_time){
  scan_time_ = scan_time;
}

void DepthImageToLaserScan::set_range_limits(const float range_min, const float range_max){
  range_min_ = range_min;
  range_max_ = range_max;
}

void DepthImageToLaserScan::set_scan_height(const int scan_height){
  scan_height_ = scan_height;
}

void DepthImageToLaserScan::set_output_frame(const std::string& output_frame_id){
  output_frame_id_ = output_frame_id;
}

const std::string& DepthImageToLaserScan::get_output_frame() {
	return output_frame_id_;
}

void DepthImageToLaserScan::set_scan_tilt(const int scan_tilt) {
	scan_tilt_ = scan_tilt;
}
