// obstacle_to_px4.cpp
// PX4 1.17 定点模式"刹车式防撞"数据发布节点
// 功能: 点云 -> 机体系 -> 高度带滤波 -> 水平扇区最近距离 -> LaserScan -> /mavros/obstacle/send
// 机型 A: /odin1/cloud_raw (前向 120°x90°)   机型 B: /livox/lidar (360°)
//
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <mutex>
#include <vector>
#include <string>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <limits>
#include <cstring>

class ObstacleToPx4
{
public:
  ObstacleToPx4()
    : nh_("~")
    , tf_buffer_()
    , tf_listener_(tf_buffer_)
  {
    // ---------- 参数 ----------
    nh_.param<std::string>("cloud_topic", cloud_topic_, "/livox/lidar");
    nh_.param<std::string>("target_frame", target_frame_, "body");
    nh_.param<int>("num_bins", num_bins_, 36);
    double h_fov_deg = 360.0;
    nh_.param<double>("h_fov_deg", h_fov_deg, 360.0);
    nh_.param<double>("height_band", height_band_, 1.5);
    nh_.param<double>("z_min", z_min_, -0.05);  // ?????????(m), ?????0.07m
    nh_.param<double>("range_min", range_min_, 0.2);
    nh_.param<double>("range_max", range_max_, 20.0);
    nh_.param<double>("accumulation_window", accum_window_, 0.2);
    nh_.param<double>("publish_rate", publish_rate_, 20.0);
    nh_.param<double>("max_cloud_age", max_cloud_age_, 0.5);
    nh_.param<double>("angle_sign", angle_sign_, -1.0);
    nh_.param<bool>("debug_markers", debug_markers_, true);

    num_bins_ = std::max(1, std::min(num_bins_, 72));
    angle_inc_ = 2.0 * M_PI / static_cast<double>(num_bins_);
    h_fov_rad_ = h_fov_deg * M_PI / 180.0;

    bin_cur_.assign(num_bins_, range_max_);
    bin_prev_.assign(num_bins_, range_max_);
    cover_.assign(num_bins_, false);
    buildCoverMask();

    cloud_sub_ = nh_.subscribe(cloud_topic_, 5, &ObstacleToPx4::cloudCb, this);
    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/mavros/obstacle/send", 5);
    scan_debug_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan_debug", 5);  // ???: ??? TF ????
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("sector_marker", 1);
    status_pub_ = nh_.advertise<std_msgs::String>("status", 1);
    timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), &ObstacleToPx4::publishCb, this);

    ROS_INFO("[obstacle_to_px4] cloud=%s target=%s bins=%d h_fov=%.1fdeg band=%.2fm "
             "range=[%.2f,%.2f] rate=%.1fHz",
             cloud_topic_.c_str(), target_frame_.c_str(), num_bins_, h_fov_deg,
             height_band_, range_min_, range_max_, publish_rate_);
  }

private:
  // 覆盖掩码: 以 0°=机头为中心, 覆盖 ±h_fov/2 的扇区才有数据
  void buildCoverMask()
  {
    for (int i = 0; i < num_bins_; ++i)
    {
      double center = angle_inc_ * (static_cast<double>(i) + 0.5);
      if (center > M_PI) center -= 2.0 * M_PI;
      cover_[i] = (std::fabs(center) <= h_fov_rad_ / 2.0 + angle_inc_ / 2.0);
    }
  }

  // 点云回调: 变换到机体系 -> 高度带 -> 扇区最近距离(带时间窗累积)
  void cloudCb(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    if (msg->header.frame_id.empty())
    {
      ROS_WARN_THROTTLE(2.0, "[obstacle_to_px4] cloud has empty frame_id, ignored");
      return;
    }

    geometry_msgs::TransformStamped ts;
    try
    {
      ts = tf_buffer_.lookupTransform(target_frame_, msg->header.frame_id,
                                      msg->header.stamp, ros::Duration(0.05));
    }
    catch (tf2::TransformException &)
    {
      // 传感器设备时间戳与 ROS 时间存在偏移时, 回退到"最新可用变换"
      try
      {
        ts = tf_buffer_.lookupTransform(target_frame_, msg->header.frame_id,
                                        ros::Time(0), ros::Duration(0.05));
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN_THROTTLE(2.0, "[obstacle_to_px4] TF %s->%s failed: %s",
                          target_frame_.c_str(), msg->header.frame_id.c_str(), ex.what());
        return;
      }
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN_THROTTLE(2.0, "[obstacle_to_px4] TF %s->%s failed: %s",
                        target_frame_.c_str(), msg->header.frame_id.c_str(), ex.what());
      return;
    }

    const geometry_msgs::Quaternion &q = ts.transform.rotation;
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 rot(quat);
    const geometry_msgs::Vector3 &tv = ts.transform.translation;

    std::lock_guard<std::mutex> lock(mtx_);

    // 手工按字段偏移遍历, 避免对迭代器实现的依赖
    uint32_t ox = 0, oy = 0, oz = 0;
    bool has_xyz = false;
    for (const sensor_msgs::PointField &f : msg->fields)
    {
      if (f.name == "x" || f.name == "y" || f.name == "z")
      {
        has_xyz = true;
        if (f.name == "x") ox = f.offset;
        else if (f.name == "y") oy = f.offset;
        else oz = f.offset;
      }
    }
    if (!has_xyz)
    {
      ROS_WARN_THROTTLE(2.0, "[obstacle_to_px4] cloud has no x/y/z fields, ignored");
      return;
    }

    const size_t n_pts = static_cast<size_t>(msg->width) * msg->height;
    const uint32_t ps = msg->point_step;
    const uint8_t *dptr = msg->data.data();
    size_t n_processed = 0;
    double first_fx = 0.0, first_fy = 0.0, first_fz = std::numeric_limits<float>::quiet_NaN();
    bool have_first = false;

    // 逐帧处理: 当前帧结果存入 bin_cur_, 上一帧保留在 bin_prev_
    const ros::Time now = ros::Time::now();
    cloud_ts_ = msg->header.stamp;  // ??/????, ?????
    bin_prev_ = bin_cur_;
    std::fill(bin_cur_.begin(), bin_cur_.end(), range_max_);
    frame_ts_ = now;

    for (size_t i = 0; i < n_pts; ++i)
    {
      const uint8_t *p = dptr + i * ps;
      float fx = 0.0f, fy = 0.0f, fz = 0.0f;
      std::memcpy(&fx, p + ox, sizeof(float));
      std::memcpy(&fy, p + oy, sizeof(float));
      std::memcpy(&fz, p + oz, sizeof(float));
      if (!have_first)
      {
        first_fx = fx; first_fy = fy; first_fz = fz;
        have_first = true;
      }
      if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(fz))
        continue;

      tf2::Vector3 pb = rot * tf2::Vector3(fx, fy, fz)
                        + tf2::Vector3(tv.x, tv.y, tv.z);

      // 高度带: 只保留机体高度附近的点(过滤地面/高空)
      if (pb.z() < z_min_ || pb.z() > height_band_)  // ????(????)???
        continue;

      const double d = std::hypot(pb.x(), pb.y());
      if (d < range_min_ || d > range_max_)
        continue;

      // 水平角: 0°=机头; angle_sign 对齐 PX4 "俯视顺时针为正"约定
      double a = std::atan2(pb.y(), pb.x()) * angle_sign_;
      if (a < 0.0) a += 2.0 * M_PI;
      if (a >= 2.0 * M_PI) a -= 2.0 * M_PI;

      const int bin = static_cast<int>(a / angle_inc_) % num_bins_;
      if (d < bin_cur_[bin])
      {
        bin_cur_[bin] = d;
        ++n_processed;
      }
    }
    ROS_WARN_THROTTLE(5.0,
                      "[obstacle_to_px4] cloud pts=%zu processed=%zu first=(%.2f,%.2f,%.2f)",
                      n_pts, n_processed, first_fx, first_fy, first_fz);
    last_cloud_ = now;
  }

  // 定时发布: 组装 LaserScan -> /mavros/obstacle/send
  void publishCb(const ros::TimerEvent &)
  {
    const ros::Time now = ros::Time::now();
    std::lock_guard<std::mutex> lock(mtx_);

    // 失效保护: 点云超时则停止发布, 交给 PX4 刹车(0.5s锁xy / 5s HOLD)
    const double age = last_cloud_.isZero()
                       ? std::numeric_limits<double>::infinity()
                       : (now - last_cloud_).toSec();
    if (age > max_cloud_age_)
    {
      if (last_cloud_.isZero())
        ROS_WARN_THROTTLE(2.0, "[obstacle_to_px4] no cloud received yet, stop publishing");
      else
        ROS_WARN_THROTTLE(2.0, "[obstacle_to_px4] cloud stale (%.2fs > %.2fs), stop publishing",
                          age, max_cloud_age_);
      return;
    }

    sensor_msgs::LaserScan scan;
    scan.header.stamp = now;
    scan.header.frame_id = target_frame_;
    scan.angle_min = 0.0;
    scan.angle_increment = angle_inc_;
    scan.angle_max = 2.0 * M_PI - angle_inc_;
    scan.range_min = range_min_;
    scan.range_max = range_max_;
    scan.ranges.resize(num_bins_, 0.0);

    const bool window_ok = !frame_ts_.isZero() &&
                           (now - frame_ts_).toSec() <= accum_window_;
    double min_dist = std::numeric_limits<double>::infinity();
    int no_cover_cnt = 0;
    for (int i = 0; i < num_bins_; ++i)
    {
      if (!cover_[i])
      {
        // 超出传感器 FOV: 填 NaN -> MAVROS 转 UINT16_MAX(PX4 视为未知); 0 在 MAVLink 里是障碍贴脸不可用
        scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
        ++no_cover_cnt;
        continue;
      }
      if (!window_ok)
      {
        // 有覆盖但无障碍: 填 range_max(该方向畅通)
        scan.ranges[i] = range_max_;
      }
      else
      {
        const double eff = std::min(bin_cur_[i], bin_prev_[i]);
        scan.ranges[i] = std::max(range_min_, std::min(eff, range_max_));
        if (eff < min_dist)
          min_dist = eff;
      }
    }
    scan_pub_.publish(scan);

    if (debug_markers_)
    {
      sensor_msgs::LaserScan scan_debug = scan;
      scan_debug.header.stamp = cloud_ts_;  // ??? TF ???, ? RViz ??
      scan_debug_pub_.publish(scan_debug);
    }

    ROS_WARN_THROTTLE(2.0,
                      "[obstacle_to_px4] pub bin0=%.2f bin4=%.2f bin31=%.2f min=%.2f",
                      scan.ranges[0], scan.ranges[4], scan.ranges[31],
                      std::isfinite(min_dist) ? min_dist : -1.0);

    if (debug_markers_)
      publishMarkers(now);

    std_msgs::String status;
    char buf[160];
    std::snprintf(buf, sizeof(buf),
                  "min_dist=%.2f m  no_cover=%d/%d  cloud_age=%.2fs",
                  std::isfinite(min_dist) ? min_dist : -1.0,
                  no_cover_cnt, num_bins_, age);
    status.data = buf;
    status_pub_.publish(status);
  }

  // 调试可视化: 每扇区一条线段, 近红远绿
  void publishMarkers(const ros::Time &now)
  {
    visualization_msgs::MarkerArray arr;
    for (int i = 0; i < num_bins_; ++i)
    {
      visualization_msgs::Marker m;
      m.header.stamp = cloud_ts_;  // ??? TF ???, ? RViz ??
      m.header.frame_id = target_frame_;
      m.ns = "sectors";
      m.id = i;
      m.type = visualization_msgs::Marker::LINE_STRIP;
      m.action = visualization_msgs::Marker::ADD;
      m.scale.x = 0.03;
      m.color.a = 0.8;

      const bool fresh = cover_[i] && !frame_ts_.isZero() &&
                         (now - frame_ts_).toSec() <= accum_window_;
      const double d = fresh ? std::min(std::min(bin_cur_[i], bin_prev_[i]), range_max_)
                             : range_max_;

      // 把"扫描角"换算回物理方向画 marker(与扫描数据方向一致)
      const double aphys = angle_inc_ * (static_cast<double>(i) + 0.5) / angle_sign_;
      geometry_msgs::Point p0, p1;
      p0.x = 0.0; p0.y = 0.0; p0.z = 0.0;
      p1.x = std::cos(aphys) * d;
      p1.y = std::sin(aphys) * d;
      p1.z = 0.0;
      m.points.push_back(p0);
      m.points.push_back(p1);

      const double k = (d - range_min_) / (range_max_ - range_min_);
      m.color.r = 1.0 - k;
      m.color.g = k;
      m.color.b = 0.0;
      arr.markers.push_back(m);
    }
    marker_pub_.publish(arr);
  }

  ros::NodeHandle nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string cloud_topic_, target_frame_;
  int num_bins_;
  double h_fov_rad_, angle_inc_, height_band_, z_min_, range_min_, range_max_;
  double accum_window_, publish_rate_, max_cloud_age_, angle_sign_;
  bool debug_markers_;

    std::vector<double> bin_cur_, bin_prev_;
    std::vector<bool> cover_;
    ros::Time frame_ts_;
    ros::Time cloud_ts_;
    std::mutex mtx_;
  ros::Time last_cloud_;

  ros::Subscriber cloud_sub_;
  ros::Publisher scan_pub_, scan_debug_pub_, marker_pub_, status_pub_;
  ros::Timer timer_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle_to_px4");
  ObstacleToPx4 node;
  ros::spin();
  return 0;
}
