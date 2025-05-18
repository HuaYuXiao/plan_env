/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/



#ifndef _SDF_MAP_H
#define _SDF_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <random>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <ros/ros.h>
#include <tuple>
#include <visualization_msgs/Marker.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <plan_env/raycast.h>
#include <plan_env/map_ros.h>
#include <chrono>
#include <plan_env/matrix_hash.h>

#define logit(x) (log((x) / (1 - (x))))

using namespace std;

namespace fast_planner {
// constant parameters

struct MappingParameters {
    /* map properties */
    Eigen::Vector3d map_origin_, map_size_;
    Eigen::Vector3d map_min_boundary_, map_max_boundary_;  // map range in pos
    Eigen::Vector3i map_voxel_num_;                        // map range in index
    Eigen::Vector3d local_update_range_;
    double resolution_, resolution_inv_;
    double obstacles_inflation_;
    string frame_id_;
    int pose_type_;

    /* camera parameters */
    double cx_, cy_, fx_, fy_;

    /* time out */
    double odom_depth_timeout_;

    /* depth image projection filtering */
    double depth_filter_maxdist_, depth_filter_mindist_, depth_filter_tolerance_;
    int depth_filter_margin_;
    bool use_depth_filter_;
    double k_depth_scaling_factor_;
    int skip_pixel_;

    /* raycasting */
    double p_hit_, p_miss_, p_min_, p_max_, p_occ_;  // occupancy probability
    double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,
            min_occupancy_log_;                   // logit of occupancy probability
    double min_ray_length_, max_ray_length_;  // range of doing raycasting

    /* local map update and clear */
    double local_bound_inflate_;
    int local_map_margin_;

    /* visualization and computation time display */
    double esdf_slice_height_, visualization_truncate_height_, virtual_ceil_height_, ground_height_, virtual_ceil_yp_, virtual_ceil_yn_;
    bool show_esdf_time_, show_occ_time_;

    /* active mapping */
    double unknown_flag_;
};

// intermediate mapping data for fusion, esdf

struct MappingData {
    // main map data, occupancy of each voxel and Euclidean distance

    std::vector<double> occupancy_buffer_;
    std::vector<char> occupancy_buffer_neg;
    std::vector<char> occupancy_buffer_inflate_;
    std::vector<double> distance_buffer_;
    std::vector<double> distance_buffer_neg_;
    std::vector<double> distance_buffer_all_;
    std::vector<double> tmp_buffer1_;
    std::vector<double> tmp_buffer2_;

    // camera position and pose data

    Eigen::Vector3d camera_pos_, last_camera_pos_;
    Eigen::Matrix3d camera_r_m_, last_camera_r_m_;
    Eigen::Matrix4d cam2body_;

    // depth image data

    cv::Mat depth_image_, last_depth_image_;
    int image_cnt_;

    // flags of map state

    bool occ_need_update_, local_updated_, esdf_need_update_;
    bool has_first_depth_;
    bool has_odom_, has_cloud_;

    // odom_depth_timeout_
    ros::Time last_occ_update_time_;
    bool flag_depth_odom_timeout_;
    bool flag_use_depth_fusion;

    // depth image projected point cloud

    vector<Eigen::Vector3d> proj_points_;
    int proj_points_cnt;

    // flag buffers for speeding up raycasting

    vector<short> count_hit_, count_hit_and_miss_;
    vector<char> flag_traverse_, flag_rayend_;
    char raycast_num_;
    queue<Eigen::Vector3i> cache_voxel_;

    // range of updating ESDF

    Eigen::Vector3i local_bound_min_, local_bound_max_;

    // computation time

    double fuse_time_, esdf_time_, max_fuse_time_, max_esdf_time_;
    int update_num_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class SDFMap {
public:
    SDFMap() {}
    ~SDFMap() {}

    enum { POSE_STAMPED = 1, ODOMETRY = 2, INVALID_IDX = -10000 };

    // occupancy map management
    void resetBuffer();
    void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);

    inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
    inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
    inline int toAddress(const Eigen::Vector3i& id);
    inline int toAddress(int& x, int& y, int& z);
    inline bool isInMap(const Eigen::Vector3d& pos);
    inline bool isInMap(const Eigen::Vector3i& idx);

    inline void setOccupancy(Eigen::Vector3d pos, double occ = 1);
    inline void setOccupied(Eigen::Vector3d pos);
    inline int getOccupancy(Eigen::Vector3d pos);
    inline int getOccupancy(Eigen::Vector3i id);
    inline int getInflateOccupancy(Eigen::Vector3d pos);

    inline void boundIndex(Eigen::Vector3i& id);
    inline bool isUnknown(const Eigen::Vector3i& id);
    inline bool isUnknown(const Eigen::Vector3d& pos);
    inline bool isKnownFree(const Eigen::Vector3i& id);
    inline bool isKnownOccupied(const Eigen::Vector3i& id);

    // distance field management
    inline double getDistance(const Eigen::Vector3d& pos);
    inline double getDistance(const Eigen::Vector3i& id);
    inline double getDistWithGradTrilinear(Eigen::Vector3d pos, Eigen::Vector3d& grad);
    void getSurroundPts(const Eigen::Vector3d& pos, Eigen::Vector3d pts[2][2][2], Eigen::Vector3d& diff);
    // /inline void setLocalRange(Eigen::Vector3d min_pos, Eigen::Vector3d
    // max_pos);

    void updateESDF3d();
    void getSliceESDF(const double height, const double res, const Eigen::Vector4d& range,
                      vector<Eigen::Vector3d>& slice, vector<Eigen::Vector3d>& grad,
                      int sign = 1);  // 1 pos, 2 neg, 3 combined

    void initMap(ros::NodeHandle& nh);

    void publishMap();
    void publishMapInflate(bool all_info = false);
    void publishESDF();
    void publishUpdateRange();

    void publishUnknown();
    void publishDepth();

    void checkDist();
    bool hasDepthObservation();
    bool odomValid();
    void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size);
    double getResolution();
    Eigen::Vector3d getOrigin();
    int getVoxelNum();
    bool getOdomDepthTimeout() { return md_.flag_depth_odom_timeout_; }

    typedef std::shared_ptr<SDFMap> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    MappingParameters mp_;
    MappingData md_;

    template <typename F_get_val, typename F_set_val>
    void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);

    // get depth image and camera pose
    void depthPoseCallback(const sensor_msgs::ImageConstPtr& img,
                           const geometry_msgs::PoseStampedConstPtr& pose);
    void extrinsicCallback(const nav_msgs::OdometryConstPtr& odom);
    void depthOdomCallback(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom);
    void depthCallback(const sensor_msgs::ImageConstPtr& img);
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& img);
    void poseCallback(const geometry_msgs::PoseStampedConstPtr& pose);
    void odomCallback(const nav_msgs::OdometryConstPtr& odom);

    // update occupancy by raycasting, and update ESDF
    void updateOccupancyCallback(const ros::TimerEvent& /*event*/);
    void updateESDFCallback(const ros::TimerEvent& /*event*/);
    void visCallback(const ros::TimerEvent& /*event*/);

    // main update process
    void projectDepthImage();
    void raycastProcess();
    void clearAndInflateLocalMap();

    inline void inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts);
    int setCacheOccupancy(Eigen::Vector3d pos, int occ);
    Eigen::Vector3d closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt);

    // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
    // nav_msgs::Odometry> SyncPolicyImageOdom; typedef
    // message_filters::sync_policies::ExactTime<sensor_msgs::Image,
    // geometry_msgs::PoseStamped> SyncPolicyImagePose;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
            SyncPolicyImageOdom;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped>
            SyncPolicyImagePose;
    typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
    typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;

    ros::NodeHandle nh;
    shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
    shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
    shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
    SynchronizerImagePose sync_image_pose_;
    SynchronizerImageOdom sync_image_odom_;

    ros::Subscriber indep_depth_sub_, indep_odom_sub_, indep_pose_sub_, indep_cloud_sub_, extrinsic_sub_;
    ros::Publisher map_pub_, esdf_pub_, map_inf_pub_, update_range_pub_;
    ros::Publisher unknown_pub_, depth_pub_;
    ros::Timer occ_timer_, esdf_timer_, vis_timer_;

    //
    uniform_real_distribution<double> rand_noise_;
    normal_distribution<double> rand_noise2_;
    default_random_engine eng_;
};

/* ============================== definition of inline function
 * ============================== */

inline int SDFMap::toAddress(const Eigen::Vector3i& id) {
    return id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + id(1) * mp_.map_voxel_num_(2) + id(2);
}

inline int SDFMap::toAddress(int& x, int& y, int& z) {
    return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + y * mp_.map_voxel_num_(2) + z;
}

inline void SDFMap::boundIndex(Eigen::Vector3i& id) {
    Eigen::Vector3i id1;
    id1(0) = max(min(id(0), mp_.map_voxel_num_(0) - 1), 0);
    id1(1) = max(min(id(1), mp_.map_voxel_num_(1) - 1), 0);
    id1(2) = max(min(id(2), mp_.map_voxel_num_(2) - 1), 0);
    id = id1;
}

inline double SDFMap::getDistance(const Eigen::Vector3d& pos) {
    Eigen::Vector3i id;
    posToIndex(pos, id);
    boundIndex(id);

    return md_.distance_buffer_all_[toAddress(id)];
}

inline double SDFMap::getDistance(const Eigen::Vector3i& id) {
    Eigen::Vector3i id1 = id;
    boundIndex(id1);
    return md_.distance_buffer_all_[toAddress(id1)];
}

inline bool SDFMap::isUnknown(const Eigen::Vector3i& id) {
    Eigen::Vector3i id1 = id;
    boundIndex(id1);
    return md_.occupancy_buffer_[toAddress(id1)] < mp_.clamp_min_log_ - 1e-3;
}

inline bool SDFMap::isUnknown(const Eigen::Vector3d& pos) {
    Eigen::Vector3i idc;
    posToIndex(pos, idc);
    return isUnknown(idc);
}

inline bool SDFMap::isKnownFree(const Eigen::Vector3i& id) {
    Eigen::Vector3i id1 = id;
    boundIndex(id1);
    int adr = toAddress(id1);

    // return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ &&
    //     md_.occupancy_buffer_[adr] < mp_.min_occupancy_log_;
    return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ && md_.occupancy_buffer_inflate_[adr] == 0;
}

inline bool SDFMap::isKnownOccupied(const Eigen::Vector3i& id) {
    Eigen::Vector3i id1 = id;
    boundIndex(id1);
    int adr = toAddress(id1);

    return md_.occupancy_buffer_inflate_[adr] == 1;
}

inline double SDFMap::getDistWithGradTrilinear(Eigen::Vector3d pos, Eigen::Vector3d& grad) {
    if (!isInMap(pos)) {
        grad.setZero();
        return 0;
    }

    /* use trilinear interpolation */
    Eigen::Vector3d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector3d::Ones();

    Eigen::Vector3i idx;
    posToIndex(pos_m, idx);

    Eigen::Vector3d idx_pos, diff;
    indexToPos(idx, idx_pos);

    diff = (pos - idx_pos) * mp_.resolution_inv_;

    double values[2][2][2];
    for (int x = 0; x < 2; x++) {
        for (int y = 0; y < 2; y++) {
            for (int z = 0; z < 2; z++) {
                Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
                values[x][y][z] = getDistance(current_idx);
            }
        }
    }

    double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
    double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
    double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
    double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
    double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
    double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
    double dist = (1 - diff[2]) * v0 + diff[2] * v1;

    grad[2] = (v1 - v0) * mp_.resolution_inv_;
    grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * mp_.resolution_inv_;
    grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
    grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
    grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
    grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);

    grad[0] *= mp_.resolution_inv_;

    return dist;
}

inline void SDFMap::setOccupied(Eigen::Vector3d pos) {
    if (!isInMap(pos)) return;

    Eigen::Vector3i id;
    posToIndex(pos, id);

    md_.occupancy_buffer_inflate_[id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
                                  id(1) * mp_.map_voxel_num_(2) + id(2)] = 1;
}

inline void SDFMap::setOccupancy(Eigen::Vector3d pos, double occ) {
    if (occ != 1 && occ != 0) {
        cout << "occ value error!" << endl;
        return;
    }

    if (!isInMap(pos)) return;

    Eigen::Vector3i id;
    posToIndex(pos, id);

    md_.occupancy_buffer_[toAddress(id)] = occ;
}

inline int SDFMap::getOccupancy(Eigen::Vector3d pos) {
    if (!isInMap(pos)) return -1;

    Eigen::Vector3i id;
    posToIndex(pos, id);

    return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline int SDFMap::getInflateOccupancy(Eigen::Vector3d pos) {
    if (!isInMap(pos)) return -1;

    Eigen::Vector3i id;
    posToIndex(pos, id);

    return int(md_.occupancy_buffer_inflate_[toAddress(id)]);
}

inline int SDFMap::getOccupancy(Eigen::Vector3i id) {
    if (id(0) < 0 || id(0) >= mp_.map_voxel_num_(0) || id(1) < 0 || id(1) >= mp_.map_voxel_num_(1) ||
        id(2) < 0 || id(2) >= mp_.map_voxel_num_(2))
        return -1;

    return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline bool SDFMap::isInMap(const Eigen::Vector3d& pos) {
    if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
        pos(2) < mp_.map_min_boundary_(2) + 1e-4) {
        // cout << "less than min range!" << endl;
        return false;
    }
    if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4 ||
        pos(2) > mp_.map_max_boundary_(2) - 1e-4) {
        return false;
    }
    return true;
}

inline bool SDFMap::isInMap(const Eigen::Vector3i& idx) {
    if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0) {
        return false;
    }
    if (idx(0) > mp_.map_voxel_num_(0) - 1 || idx(1) > mp_.map_voxel_num_(1) - 1 ||
        idx(2) > mp_.map_voxel_num_(2) - 1) {
        return false;
    }
    return true;
}

inline void SDFMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
    for (int i = 0; i < 3; ++i) id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
}

inline void SDFMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
    for (int i = 0; i < 3; ++i) pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}

inline void SDFMap::inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts) {
    int num = 0;

    /* ---------- + shape inflate ---------- */
    // for (int x = -step; x <= step; ++x)
    // {
    //   if (x == 0)
    //     continue;
    //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
    // }
    // for (int y = -step; y <= step; ++y)
    // {
    //   if (y == 0)
    //     continue;
    //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
    // }
    // for (int z = -1; z <= 1; ++z)
    // {
    //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
    // }

    /* ---------- all inflate ---------- */
    for (int x = -step; x <= step; ++x)
        for (int y = -step; y <= step; ++y)
            for (int z = -step; z <= step; ++z) {
                pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
            }
}

} // namespace fast_planner

/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the header file of SDFMap class, which implements
 *                   volumetric mapping module in FC-Planner.
 * License      :    GNU General Public License <http://www.gnu.org/licenses/>.
 * Project      :    FC-Planner is free software: you can redistribute it and/or 
 *                   modify it under the terms of the GNU Lesser General Public 
 *                   License as published by the Free Software Foundation, 
 *                   either version 3 of the License, or (at your option) any 
 *                   later version.
 *                   FC-Planner is distributed in the hope that it will be useful,
 *                   but WITHOUT ANY WARRANTY; without even the implied warranty 
 *                   of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 *                   See the GNU General Public License for more details.
 * Website      :    https://hkust-aerial-robotics.github.io/FC-Planner/
 *⭐⭐⭐*****************************************************************⭐⭐⭐*/

namespace cv {
class Mat;
}

class RayCaster;

namespace predrecon {
struct MapParam;
struct MapData;
struct HCMapParam;
struct HCMapData;
class MapROS;

class SDFMap {
public:
  SDFMap();
  ~SDFMap();

  enum OCCUPANCY { UNKNOWN, FREE, OCCUPIED, PRED_INTERNAL, PRED_EXTERNAL, HC_INTERNAL };

  std::vector<int> recon_poses_idx_;
  std::vector<int> recon_states_;
  Eigen::Vector3d min_bound, max_bound;

  void initMap(ros::NodeHandle& nh);
  void inputPointCloud(const pcl::PointCloud<pcl::PointXYZ>& points, const int& point_num,
                       const Eigen::Vector3d& camera_pos);
  void inputPredictionCloud(const pcl::PointCloud<pcl::PointXYZ>& points, const int& point_num,
                       const Eigen::Vector3d& viewpoint);
  void inputFreePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& points);

  void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
  void posToIndex_hc(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
  void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
  void indexToPos_hc(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
  void boundIndex(Eigen::Vector3i& id);
  int toAddress(const Eigen::Vector3i& id);
  int toAddress(const int& x, const int& y, const int& z);
  int toAddress_hc(const Eigen::Vector3i& id);
  int toAddress_hc(const int& x, const int& y, const int& z);
  bool isInMap(const Eigen::Vector3d& pos);
  bool isInMap(const Eigen::Vector3i& idx);
  bool isInBox(const Eigen::Vector3i& id);
  bool isInBox(const Eigen::Vector3d& pos);
  void boundBox(Eigen::Vector3d& low, Eigen::Vector3d& up);
  int getOccupancy(const Eigen::Vector3d& pos);
  int getOccupancy(const Eigen::Vector3i& id);
  void setOccupied(const Eigen::Vector3d& pos, const int& occ = 1);
  int getInflateOccupancy(const Eigen::Vector3d& pos);
  int getInflateOccupancy(const Eigen::Vector3i& id);
  double getDistance(const Eigen::Vector3d& pos);
  double getDistance(const Eigen::Vector3i& id);
  double getDistWithGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad);
  void updateESDF3d();
  void resetBuffer();
  void resetBuffer(const Eigen::Vector3d& min, const Eigen::Vector3d& max);

  // prediction related
  int get_PredStates(const Eigen::Vector3d& pos);
  int get_PredStates(const Eigen::Vector3i& id);
  int get_qualified(const Eigen::Vector3i& id, const int& step);
  void reset_PredStates();

  void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size);
  void getBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax);
  void getUpdatedBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax, bool reset = false);
  double getResolution();
  int getVoxelNum();
  void getOccMap(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  void inv_address(int& idx, Eigen::Vector3i& pos);
  void inflate_pred(Eigen::Vector3i& idx);
  bool getInternal_check(const Eigen::Vector3d& pos, const int& step);
  bool setcenter_check(const Eigen::Vector3d& pos, const int& step);
  bool setcenter_check(const Eigen::Vector3i& pos, const int& step);

  // ! /* hierarchical_coverage_planner */ --------------------------------------------
  bool zFlag; double zPos;
  int checkSize, inflate_num;
  void initHCMap(ros::NodeHandle& nh, pcl::PointCloud<pcl::PointXYZ>::Ptr& model);
  void InternalSpace(map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>& seg_cloud, Eigen::MatrixXd& vertices, map<int, vector<int>>& segments);
  void SetOcc(pcl::PointCloud<pcl::PointXYZ>::Ptr& OccModel);
  void OuterCheck(vector<Eigen::VectorXd>& outers);
  vector<Eigen::Vector3d> points_in_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud, Eigen::Vector3d& point_on_plane, Eigen::Vector3d& plane_normal, double thickness);
  bool isInMap_hc(const Eigen::Vector3d& pos);
  bool isInMap_hc(const Eigen::Vector3i& idx);
  bool safety_check(Eigen::Vector3d& pos);
  bool safety_check(Eigen::Vector3i& id);
  bool occCheck(Eigen::Vector3d& pos);
  bool occCheck(Eigen::Vector3i& id);
  bool freeCheck(Eigen::Vector3d& pos);
  bool freeCheck(Eigen::Vector3i& id);
  int get_Internal(const Eigen::Vector3d& pos);
  int get_Internal(const Eigen::Vector3i& id);
  void publishMap();

  unique_ptr<HCMapParam> hcmp_;
  unique_ptr<HCMapData> hcmd_;
  unique_ptr<RayCaster> internal_cast_;
  // ! /* hierarchical_coverage_planner */ --------------------------------------------

private:
  void clearAndInflateLocalMap();
  void inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts);
  void setCacheOccupancy(const int& adr, const int& occ);
  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt);
  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);
  
  ros::Publisher occ_pub,free_pub;
  
  unique_ptr<MapParam> mp_;
  unique_ptr<MapData> md_;
  unique_ptr<MapROS> mr_;
  unique_ptr<RayCaster> caster_;

  friend MapROS;

public:
  typedef std::shared_ptr<SDFMap> Ptr;
  typedef std::unique_ptr<SDFMap> HCPtr;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct MapParam {
  // map properties
  Eigen::Vector3d map_origin_, map_size_;
  Eigen::Vector3d map_min_boundary_, map_max_boundary_;
  Eigen::Vector3i map_voxel_num_;
  double resolution_, resolution_inv_;
  double obstacles_inflation_;
  double virtual_ceil_height_, ground_height_;
  Eigen::Vector3i box_min_, box_max_;
  Eigen::Vector3d box_mind_, box_maxd_;
  double default_dist_;
  bool optimistic_, signed_dist_;
  // map fusion
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;  // occupancy probability
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_, min_occupancy_log_;  // logit
  double max_ray_length_;
  double max_ray_length_prediction;
  double local_bound_inflate_;
  int local_map_margin_;
  double unknown_flag_;
  double view_thre_;
};

struct HCMapParam
{
  double resolution_, resolution_inv_, proj_interval, thickness, size_inflate;
  double checkScale;
  Eigen::Vector3d map_origin_, map_size_;
  Eigen::Vector3d map_min_boundary_, map_max_boundary_;
  Eigen::Vector3i map_voxel_num_;
  Eigen::Vector3i box_min_, box_max_;
  Eigen::Vector3d box_mind_, box_maxd_;
  Eigen::Vector3i map_origin_idx_;
};

struct MapData {
  // main map data, occupancy of each voxel and Euclidean distance
  std::vector<double> occupancy_buffer_;
  std::vector<char> occupancy_buffer_inflate_;
  // prediction map states
  std::vector<char> occupancy_buffer_pred_;
  std::vector<double> distance_buffer_neg_;
  std::vector<double> distance_buffer_;
  std::vector<double> tmp_buffer1_;
  std::vector<double> tmp_buffer2_;
  // data for updating
  vector<short> count_hit_, count_miss_, count_hit_and_miss_;
  vector<char> flag_rayend_, flag_visited_;
  char raycast_num_;
  queue<int> cache_voxel_;
  Eigen::Vector3i local_bound_min_, local_bound_max_;
  Eigen::Vector3d update_min_, update_max_;
  bool reset_updated_box_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct HCMapData
{
  std::vector<char> occupancy_buffer_hc_;
  std::vector<char> occupancy_inflate_buffer_hc_;
  std::vector<char> occupancy_buffer_internal_;
  pcl::PointCloud<pcl::PointXYZ> occ_cloud;
  pcl::PointCloud<pcl::PointXYZ> internal_cloud;
  std::vector<bool> seg_occ_visited_buffer_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

inline void SDFMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i)
    id(i) = floor((pos(i) - mp_->map_origin_(i)) * mp_->resolution_inv_);
}

inline void SDFMap::posToIndex_hc(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i)
    id(i) = floor((pos(i) - hcmp_->map_origin_(i)) * hcmp_->resolution_inv_);
}

inline void SDFMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
  for (int i = 0; i < 3; ++i)
    pos(i) = (id(i) + 0.5) * mp_->resolution_ + mp_->map_origin_(i);
}

inline void SDFMap::indexToPos_hc(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
  for (int i = 0; i < 3; ++i)
    pos(i) = (id(i) + 0.5) * hcmp_->resolution_ + hcmp_->map_origin_(i);
}


inline void SDFMap::boundIndex(Eigen::Vector3i& id) {
  Eigen::Vector3i id1;
  id1(0) = max(min(id(0), mp_->map_voxel_num_(0) - 1), 0);
  id1(1) = max(min(id(1), mp_->map_voxel_num_(1) - 1), 0);
  id1(2) = max(min(id(2), mp_->map_voxel_num_(2) - 1), 0);
  id = id1;
}

inline int SDFMap::toAddress(const int& x, const int& y, const int& z) {
  return x * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2) + y * mp_->map_voxel_num_(2) + z;
}

inline int SDFMap::toAddress(const Eigen::Vector3i& id) {
  return toAddress(id[0], id[1], id[2]);
}

inline int SDFMap::toAddress_hc(const int& x, const int& y, const int& z) {
  int xi = x-hcmp_->map_origin_idx_(0); int yi = y-hcmp_->map_origin_idx_(1); int zi = z-hcmp_->map_origin_idx_(2); 
  return xi * hcmp_->map_voxel_num_(1) * hcmp_->map_voxel_num_(2) + yi * hcmp_->map_voxel_num_(2) + zi;
}

inline int SDFMap::toAddress_hc(const Eigen::Vector3i& id) {
  return toAddress_hc(id[0], id[1], id[2]);
}

inline void SDFMap::inv_address(int& idx, Eigen::Vector3i& pos)
{
  int x_inv = int(idx/(mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2)));
  int y_inv = int((idx-x_inv*mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2))/mp_->map_voxel_num_(2));
  int z_inv = idx - x_inv * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2) - y_inv * mp_->map_voxel_num_(2);

  pos(0) = x_inv;
  pos(1) = y_inv;
  pos(2) = z_inv;
}

inline bool SDFMap::isInMap(const Eigen::Vector3d& pos) {
  if (pos(0) < mp_->map_min_boundary_(0) + 1e-4 || pos(1) < mp_->map_min_boundary_(1) + 1e-4 ||
      pos(2) < mp_->map_min_boundary_(2) + 1e-4)
    return false;
  if (pos(0) > mp_->map_max_boundary_(0) - 1e-4 || pos(1) > mp_->map_max_boundary_(1) - 1e-4 ||
      pos(2) > mp_->map_max_boundary_(2) - 1e-4)
    return false;
  return true;
}

inline bool SDFMap::isInMap(const Eigen::Vector3i& idx) {
  if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0) return false;
  if (idx(0) > mp_->map_voxel_num_(0) - 1 || idx(1) > mp_->map_voxel_num_(1) - 1 ||
      idx(2) > mp_->map_voxel_num_(2) - 1)
    return false;
  return true;
}

inline bool SDFMap::isInBox(const Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i) {
    if (id[i] < mp_->box_min_[i] || id[i] >= mp_->box_max_[i]) {
      return false;
    }
  }
  return true;
}

inline bool SDFMap::isInBox(const Eigen::Vector3d& pos) {
  for (int i = 0; i < 3; ++i) {
    if (pos[i] <= mp_->box_mind_[i] || pos[i] >= mp_->box_maxd_[i]) {
      return false;
    }
  }
  return true;
}

inline void SDFMap::boundBox(Eigen::Vector3d& low, Eigen::Vector3d& up) {
  for (int i = 0; i < 3; ++i) {
    low[i] = max(low[i], mp_->box_mind_[i]);
    up[i] = min(up[i], mp_->box_maxd_[i]);
  }
}

inline int SDFMap::getOccupancy(const Eigen::Vector3i& id) {
  if (!isInMap(id)) return -1;
  double occ = md_->occupancy_buffer_[toAddress(id)];
  if (occ < mp_->clamp_min_log_ - 1e-3) return UNKNOWN;
  if (occ > mp_->min_occupancy_log_) return OCCUPIED;
  return FREE;
}

inline int SDFMap::getOccupancy(const Eigen::Vector3d& pos) {
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return getOccupancy(id);
}

inline int SDFMap::get_PredStates(const Eigen::Vector3i& id)
{
  if (!isInMap(id)) return -1;
  if (md_->occupancy_buffer_pred_[toAddress(id)] == 0) return PRED_EXTERNAL;
  if (md_->occupancy_buffer_pred_[toAddress(id)] == 1) return PRED_INTERNAL;
}

inline int SDFMap::get_PredStates(const Eigen::Vector3d& pos)
{
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return get_PredStates(id);
}

inline bool SDFMap::isInMap_hc(const Eigen::Vector3d& pos) {
  if (pos(0) < hcmp_->map_min_boundary_(0) + 1e-4 || pos(1) < hcmp_->map_min_boundary_(1) + 1e-4 ||
      pos(2) < hcmp_->map_min_boundary_(2) + 1e-4)
    return false;
  if (pos(0) > hcmp_->map_max_boundary_(0) - 1e-4 || pos(1) > hcmp_->map_max_boundary_(1) - 1e-4 ||
      pos(2) > hcmp_->map_max_boundary_(2) - 1e-4)
    return false;
  return true;
}

inline bool SDFMap::isInMap_hc(const Eigen::Vector3i& idx) {
  if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0) return false;
  if (idx(0) > hcmp_->map_voxel_num_(0) - 1 || idx(1) > hcmp_->map_voxel_num_(1) - 1 ||
      idx(2) > hcmp_->map_voxel_num_(2) - 1)
    return false;
  return true;
}

inline bool SDFMap::safety_check(Eigen::Vector3i& id)
{
  if (!isInMap_hc(id)) return false;

  if (hcmd_->occupancy_buffer_hc_[toAddress_hc(id)] == 1 || hcmd_->occupancy_buffer_internal_[toAddress_hc(id)] == 1)
    return false;
  
  return true;
}

inline bool SDFMap::safety_check(Eigen::Vector3d& pos)
{
  // if (zFlag == true)
  // {
  //   if (pos(2) < zPos-1.0)
  //     return false;
  // }
  Eigen::Vector3i id;
  posToIndex_hc(pos, id);
  return safety_check(id);
}

inline bool SDFMap::occCheck(Eigen::Vector3i& id)
{
  if (!isInMap_hc(id)) return false;
  // int ne_size = checkSize;
  // Eigen::Vector3i idxNe;
  // for (int i=-ne_size; i<ne_size+1; i=ne_size+1)
  // {
  //   for (int j=-ne_size; j<ne_size+1; j=ne_size+1)
  //   {
  //     for (int k=-ne_size; k<ne_size+1; k=ne_size+1)
  //     {
  //       idxNe(0) = id(0) + i;
  //       idxNe(1) = id(1) + j;
  //       idxNe(2) = id(2) + k;
  //       if (hcmd_->occupancy_buffer_hc_[toAddress_hc(idxNe)] == 1)
  //         return false;
  //     }
  //   }
  // }
  if (hcmd_->occupancy_buffer_hc_[toAddress_hc(id)] == 1)
    return false;
  
  return true;
}

inline bool SDFMap::occCheck(Eigen::Vector3d& pos)
{
  Eigen::Vector3i id;
  posToIndex_hc(pos, id);
  return occCheck(id);
}

inline bool SDFMap::freeCheck(Eigen::Vector3i& id)
{
  // only for viewpoint_manager demo
  if (!isInMap_hc(id)) return false;
  if (hcmd_->occupancy_buffer_hc_[toAddress_hc(id)] == 2)
    return true;
  
  return false;
}

inline bool SDFMap::freeCheck(Eigen::Vector3d& pos)
{
  // only for viewpoint_manager demo
  Eigen::Vector3i id;
  posToIndex_hc(pos, id);
  return occCheck(id);
}

inline int SDFMap::get_Internal(const Eigen::Vector3i& id)
{
  if (!isInMap_hc(id)) return -1;
  if (hcmd_->occupancy_buffer_internal_[toAddress_hc(id)] == 1) 
    return HC_INTERNAL;
  if (hcmd_->occupancy_buffer_internal_[toAddress_hc(id)] == 0)
    return FREE;
}

inline int SDFMap::get_Internal(const Eigen::Vector3d& pos)
{
  Eigen::Vector3i id;
  posToIndex_hc(pos, id);
  return get_Internal(id);
}

inline void SDFMap::setOccupied(const Eigen::Vector3d& pos, const int& occ) {
  if (!isInMap(pos)) return;
  Eigen::Vector3i id;
  posToIndex(pos, id);
  md_->occupancy_buffer_inflate_[toAddress(id)] = occ;
}

inline int SDFMap::getInflateOccupancy(const Eigen::Vector3i& id) {
  if (!isInMap(id)) return -1;
  return int(md_->occupancy_buffer_inflate_[toAddress(id)]);
}

inline int SDFMap::getInflateOccupancy(const Eigen::Vector3d& pos) {
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return getInflateOccupancy(id);
}

inline double SDFMap::getDistance(const Eigen::Vector3i& id) {
  if (!isInMap(id)) return -1;
  return md_->distance_buffer_[toAddress(id)];
}

inline double SDFMap::getDistance(const Eigen::Vector3d& pos) {
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return getDistance(id);
}

inline void SDFMap::inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts) {
  int num = 0;

  /* ---------- + shape inflate ---------- */
  // for (int x = -step; x <= step; ++x)
  // {
  //   if (x == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
  // }
  // for (int y = -step; y <= step; ++y)
  // {
  //   if (y == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
  // }
  // for (int z = -1; z <= 1; ++z)
  // {
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
  // }

  /* ---------- all inflate ---------- */
  for (int x = -step; x <= step; ++x)
    for (int y = -step; y <= step; ++y)
      for (int z = -step; z <= step; ++z) {
        pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
      }
}

inline void SDFMap::inflate_pred(Eigen::Vector3i& idx)
{
  md_->occupancy_buffer_pred_[toAddress(idx)] = 1;
  md_->occupancy_buffer_pred_[toAddress(Eigen::Vector3i(idx(0) + 1, idx(1) + 0, idx(2) + 0))] = 1;
  md_->occupancy_buffer_pred_[toAddress(Eigen::Vector3i(idx(0) + 0, idx(1) + 1, idx(2) + 0))] = 1;
  md_->occupancy_buffer_pred_[toAddress(Eigen::Vector3i(idx(0) + 0, idx(1) + 0, idx(2) + 1))] = 1;
  md_->occupancy_buffer_pred_[toAddress(Eigen::Vector3i(idx(0) - 1, idx(1) + 0, idx(2) + 0))] = 1;
  md_->occupancy_buffer_pred_[toAddress(Eigen::Vector3i(idx(0) + 0, idx(1) - 0, idx(2) + 0))] = 1;
  md_->occupancy_buffer_pred_[toAddress(Eigen::Vector3i(idx(0) + 0, idx(1) + 0, idx(2) - 1))] = 1;
}

inline bool SDFMap::getInternal_check(const Eigen::Vector3d& pos, const int& step)
{
  int flag = 0;
  Eigen::Vector3i inflate_idx, id;
  posToIndex(pos, id);
  int inflate_address;
  for (int x = -step; x <= step; ++x)
    for (int y = -step; y <= step; ++y)
      for (int z = -step; z <= step; ++z) {
        inflate_idx(0) = id(0) + x;
        inflate_idx(1) = id(1) + y;
        inflate_idx(2) = id(2) + z;
        if (isInMap(inflate_idx))
        {
          inflate_address = toAddress(inflate_idx);
          if (get_PredStates(inflate_idx) == SDFMap::OCCUPANCY::PRED_INTERNAL)
          {
            flag = 1;
            break;
          }
        }
      }
  
  return flag;
}

inline bool SDFMap::setcenter_check(const Eigen::Vector3d& pos, const int& step)
{
  int all = 0, hit = 0;
  bool flag = false;
  Eigen::Vector3i id, inflate_idx;
  posToIndex(pos, id);
  // x axis
  for (int x = -step; x<= step; ++x)
  {
    inflate_idx(0) = id(0) + 2*x;
    inflate_idx(1) = id(1);
    inflate_idx(2) = id(2);
    if (isInMap(inflate_idx))
    {
      all++;
      if (get_PredStates(inflate_idx) == SDFMap::OCCUPANCY::PRED_INTERNAL)
      {
        hit++;
      }
    }
  }
  // y axis
  for (int y = -step; y<= step; ++y)
  {
    inflate_idx(0) = id(0);
    inflate_idx(1) = id(1) + 2*y;
    inflate_idx(2) = id(2);
    if (isInMap(inflate_idx))
    {
      all++;
      if (get_PredStates(inflate_idx) == SDFMap::OCCUPANCY::PRED_INTERNAL)
      {
        hit++;
      }
    }
  }
  // z axis
  for (int z = -step; z<= step; ++z)
  {
    inflate_idx(0) = id(0);
    inflate_idx(1) = id(1);
    inflate_idx(2) = id(2) + 2*z;
    if (isInMap(inflate_idx))
    {
      all++;
      if (get_PredStates(inflate_idx) == SDFMap::OCCUPANCY::PRED_INTERNAL)
      {
        hit++;
      }
    }
  }

  double ratio = (double) hit/all;
  if (ratio > 0.8)
    flag = true;
  
  return flag;
}

inline bool SDFMap::setcenter_check(const Eigen::Vector3i& pos, const int& step)
{
  int all = 0, hit = 0;
  bool flag = false;
  Eigen::Vector3i id, inflate_idx;
  id = pos;
  // x axis
  for (int x = -step; x<= step; ++x)
  {
    inflate_idx(0) = id(0) + 2*x;
    inflate_idx(1) = id(1);
    inflate_idx(2) = id(2);
    if (isInMap(inflate_idx))
    {
      all++;
      if (get_PredStates(inflate_idx) == SDFMap::OCCUPANCY::PRED_INTERNAL)
      {
        hit++;
      }
    }
  }
  // y axis
  for (int y = -step; y<= step; ++y)
  {
    inflate_idx(0) = id(0);
    inflate_idx(1) = id(1) + 2*y;
    inflate_idx(2) = id(2);
    if (isInMap(inflate_idx))
    {
      all++;
      if (get_PredStates(inflate_idx) == SDFMap::OCCUPANCY::PRED_INTERNAL)
      {
        hit++;
      }
    }
  }
  // z axis
  for (int z = -step; z<= step; ++z)
  {
    inflate_idx(0) = id(0);
    inflate_idx(1) = id(1);
    inflate_idx(2) = id(2) + 2*z;
    if (isInMap(inflate_idx))
    {
      all++;
      if (get_PredStates(inflate_idx) == SDFMap::OCCUPANCY::PRED_INTERNAL)
      {
        hit++;
      }
    }
  }

  double ratio = (double) hit/all;
  if (ratio > 0.8)
    flag = true;
  
  return flag;
}
}
#endif
