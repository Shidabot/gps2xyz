#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kWgs84SemiMajorAxis = 6378137.0;
constexpr double kWgs84Flattening = 1.0 / 298.257223563;
constexpr double kWgs84EccentricitySquared =
    kWgs84Flattening * (2.0 - kWgs84Flattening);

double degreesToRadians(const double degrees)
{
  return degrees * kPi / 180.0;
}

struct CartesianPoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

CartesianPoint geodeticToEcef(const double latitude_deg,
                              const double longitude_deg,
                              const double altitude_m)
{
  const double latitude = degreesToRadians(latitude_deg);
  const double longitude = degreesToRadians(longitude_deg);
  const double sin_latitude = std::sin(latitude);
  const double cos_latitude = std::cos(latitude);
  const double sin_longitude = std::sin(longitude);
  const double cos_longitude = std::cos(longitude);
  const double prime_vertical_radius =
      kWgs84SemiMajorAxis /
      std::sqrt(1.0 - kWgs84EccentricitySquared * sin_latitude * sin_latitude);

  CartesianPoint point;
  point.x = (prime_vertical_radius + altitude_m) * cos_latitude * cos_longitude;
  point.y = (prime_vertical_radius + altitude_m) * cos_latitude * sin_longitude;
  point.z = (prime_vertical_radius * (1.0 - kWgs84EccentricitySquared) + altitude_m) *
            sin_latitude;
  return point;
}
}  // namespace

class GpsToXyzNode
{
public:
  GpsToXyzNode() : nh_(), private_nh_("~")
  {
    private_nh_.param<std::string>("gps_topic", gps_topic_, "/gps");
    // Preserve compatibility with the original parameter name.
    private_nh_.param<std::string>("gps_sub_topic", gps_topic_, gps_topic_);
    private_nh_.param<std::string>("path_topic", path_topic_, "/gpsTrack");
    private_nh_.param<std::string>("output_frame", output_frame_, "map");
    private_nh_.param<std::string>("output_frame_name", output_frame_, output_frame_);
    private_nh_.param<bool>("auto_origin", auto_origin_, true);
    private_nh_.param<bool>("auto_get_origin_gps", auto_origin_, auto_origin_);
    private_nh_.param<double>("origin_latitude", origin_latitude_, 0.0);
    private_nh_.param<double>("origin_latitude_value", origin_latitude_, origin_latitude_);
    private_nh_.param<double>("origin_longitude", origin_longitude_, 0.0);
    private_nh_.param<double>("origin_longitude_value", origin_longitude_, origin_longitude_);
    private_nh_.param<double>("origin_altitude", origin_altitude_, 0.0);
    private_nh_.param<double>("origin_altitude_value", origin_altitude_, origin_altitude_);
    private_nh_.param<double>("yaw_offset_deg", yaw_offset_deg_, 0.0);
    private_nh_.param<double>("z_rotate_value", yaw_offset_deg_, yaw_offset_deg_);
    private_nh_.param<bool>("use_altitude", use_altitude_, true);
    private_nh_.param<double>("min_distance_m", min_distance_m_, 0.0);
    private_nh_.param<double>("max_position_variance", max_position_variance_, 0.0);
    private_nh_.param<int>("max_path_points", max_path_points_, 10000);

    min_distance_m_ = std::max(0.0, min_distance_m_);
    max_position_variance_ = std::max(0.0, max_position_variance_);
    if (max_path_points_ < 0)
    {
      ROS_WARN("~max_path_points cannot be negative; using unlimited history.");
      max_path_points_ = 0;
    }

    path_.header.frame_id = output_frame_;
    path_publisher_ = nh_.advertise<nav_msgs::Path>(path_topic_, 1, true);
    gps_subscriber_ = nh_.subscribe(gps_topic_, 100, &GpsToXyzNode::gpsCallback, this);

    if (!auto_origin_)
    {
      if (!validCoordinates(origin_latitude_, origin_longitude_, origin_altitude_))
      {
        ROS_FATAL("Configured GNSS origin is invalid.");
        ros::shutdown();
        return;
      }
      setOrigin(origin_latitude_, origin_longitude_, origin_altitude_);
    }

    ROS_INFO_STREAM("gps_to_xyz is listening on " << gps_topic_
                    << " and publishing " << path_topic_ << " in frame "
                    << output_frame_ << ".");
  }

private:
  static bool validCoordinates(const double latitude,
                               const double longitude,
                               const double altitude)
  {
    return std::isfinite(latitude) && std::isfinite(longitude) &&
           std::isfinite(altitude) && latitude >= -90.0 && latitude <= 90.0 &&
           longitude >= -180.0 && longitude <= 180.0;
  }

  bool validFix(const sensor_msgs::NavSatFix& fix) const
  {
    const double altitude = use_altitude_ ? fix.altitude : 0.0;
    if (fix.status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX)
    {
      ROS_WARN_THROTTLE(5.0, "Ignoring GNSS message without a valid fix.");
      return false;
    }
    if (!validCoordinates(fix.latitude, fix.longitude, altitude))
    {
      ROS_WARN_THROTTLE(5.0, "Ignoring GNSS message with invalid coordinates.");
      return false;
    }
    if (max_position_variance_ > 0.0 &&
        fix.position_covariance_type != sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN)
    {
      const double largest_variance =
          std::max({fix.position_covariance[0], fix.position_covariance[4],
                    fix.position_covariance[8]});
      if (!std::isfinite(largest_variance) || largest_variance < 0.0 ||
          largest_variance > max_position_variance_)
      {
        ROS_WARN_THROTTLE(5.0, "Ignoring GNSS message above the covariance threshold.");
        return false;
      }
    }
    return true;
  }

  void setOrigin(const double latitude, const double longitude, const double altitude)
  {
    origin_latitude_ = latitude;
    origin_longitude_ = longitude;
    origin_altitude_ = altitude;
    origin_ecef_ = geodeticToEcef(latitude, longitude, altitude);

    const double latitude_rad = degreesToRadians(latitude);
    const double longitude_rad = degreesToRadians(longitude);
    sin_origin_latitude_ = std::sin(latitude_rad);
    cos_origin_latitude_ = std::cos(latitude_rad);
    sin_origin_longitude_ = std::sin(longitude_rad);
    cos_origin_longitude_ = std::cos(longitude_rad);
    origin_initialized_ = true;

    ROS_INFO_STREAM("GNSS origin set to latitude=" << latitude
                    << ", longitude=" << longitude << ", altitude=" << altitude << ".");
  }

  CartesianPoint toLocalEnu(const sensor_msgs::NavSatFix& fix) const
  {
    const double altitude = use_altitude_ ? fix.altitude : origin_altitude_;
    const CartesianPoint ecef = geodeticToEcef(fix.latitude, fix.longitude, altitude);
    const double dx = ecef.x - origin_ecef_.x;
    const double dy = ecef.y - origin_ecef_.y;
    const double dz = ecef.z - origin_ecef_.z;

    CartesianPoint enu;
    enu.x = -sin_origin_longitude_ * dx + cos_origin_longitude_ * dy;
    enu.y = -sin_origin_latitude_ * cos_origin_longitude_ * dx -
            sin_origin_latitude_ * sin_origin_longitude_ * dy +
            cos_origin_latitude_ * dz;
    enu.z = cos_origin_latitude_ * cos_origin_longitude_ * dx +
            cos_origin_latitude_ * sin_origin_longitude_ * dy +
            sin_origin_latitude_ * dz;

    const double yaw = degreesToRadians(yaw_offset_deg_);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const double east = enu.x;
    const double north = enu.y;
    enu.x = cos_yaw * east - sin_yaw * north;
    enu.y = sin_yaw * east + cos_yaw * north;
    return enu;
  }

  bool farEnoughFromLastPoint(const CartesianPoint& point) const
  {
    if (!has_last_point_ || min_distance_m_ <= 0.0)
    {
      return true;
    }
    const double dx = point.x - last_point_.x;
    const double dy = point.y - last_point_.y;
    const double dz = point.z - last_point_.z;
    return dx * dx + dy * dy + dz * dz >= min_distance_m_ * min_distance_m_;
  }

  void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& fix)
  {
    if (!validFix(*fix))
    {
      return;
    }

    const double altitude = use_altitude_ ? fix->altitude : 0.0;
    if (!origin_initialized_)
    {
      setOrigin(fix->latitude, fix->longitude, altitude);
    }

    const CartesianPoint local = toLocalEnu(*fix);
    if (!farEnoughFromLastPoint(local))
    {
      return;
    }

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = fix->header.stamp.isZero() ? ros::Time::now() : fix->header.stamp;
    pose.header.frame_id = output_frame_;
    pose.pose.position.x = local.x;
    pose.pose.position.y = local.y;
    pose.pose.position.z = local.z;
    pose.pose.orientation.w = 1.0;

    path_.poses.push_back(pose);
    if (max_path_points_ > 0 &&
        path_.poses.size() > static_cast<std::size_t>(max_path_points_))
    {
      const std::size_t excess =
          path_.poses.size() - static_cast<std::size_t>(max_path_points_);
      path_.poses.erase(path_.poses.begin(), path_.poses.begin() + excess);
    }
    path_.header.stamp = pose.header.stamp;
    path_publisher_.publish(path_);

    last_point_ = local;
    has_last_point_ = true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber gps_subscriber_;
  ros::Publisher path_publisher_;
  nav_msgs::Path path_;

  std::string gps_topic_;
  std::string path_topic_;
  std::string output_frame_;
  bool auto_origin_{true};
  bool use_altitude_{true};
  bool origin_initialized_{false};
  bool has_last_point_{false};
  int max_path_points_{10000};
  double origin_latitude_{0.0};
  double origin_longitude_{0.0};
  double origin_altitude_{0.0};
  double yaw_offset_deg_{0.0};
  double min_distance_m_{0.0};
  double max_position_variance_{0.0};
  double sin_origin_latitude_{0.0};
  double cos_origin_latitude_{1.0};
  double sin_origin_longitude_{0.0};
  double cos_origin_longitude_{1.0};
  CartesianPoint origin_ecef_;
  CartesianPoint last_point_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps_to_xyz");
  GpsToXyzNode node;
  ros::spin();
  return 0;
}

