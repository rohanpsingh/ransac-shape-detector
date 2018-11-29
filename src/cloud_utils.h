#include "common_headers.h"

PointCloudPtr cloud_passthrough(const PointCloudPtr& incloud, const std::string axis, const float min_lim, const float max_lim);
PointCloudPtr cloud_downsample(const PointCloudPtr& incloud, const float leaf);
  
void seg_ransac(const PointCloudPtr& input_cloud,
		const NormalCloudPtr& input_normals,
		const std::string model_type,
		const int max_iters,
		const float dist_thresh,
		const float rad_lim,
		const int min_seg_points,
		std::vector<pcl::PointIndices::Ptr>& inliers_vec,
		std::vector<pcl::ModelCoefficients::Ptr>& coefficients_vec,
		std::vector<PointCloudPtr>& seg_clouds);
visualization_msgs::Marker process_cylin_cloud(const PointCloudPtr& input_cloud, const int id, const std::string frame_id, const pcl::ModelCoefficients::Ptr& coeff);
jsk_recognition_msgs::BoundingBox process_plane_cloud(const PointCloudPtr& input_cloud, const int id, const std::string frame_id, const pcl::ModelCoefficients::Ptr& coeff);
