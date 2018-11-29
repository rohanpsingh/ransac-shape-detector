#include "cloud_utils.h"

PointCloudPtr cloud_passthrough(const PointCloudPtr& incloud, const std::string axis, const float min_lim, const float max_lim){

  PointCloudPtr outcloud (new PointCloud);
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (incloud);
  pass.setFilterFieldName (axis);
  pass.setFilterLimits (min_lim, max_lim);
  pass.filter (*outcloud);

  return outcloud;
}

PointCloudPtr cloud_downsample(const PointCloudPtr& incloud, const float leaf){
  
  PointCloudPtr outcloud (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (incloud);
  grid.filter (*outcloud);

  return outcloud;
}

void seg_ransac(const PointCloudPtr& input_cloud,
		const NormalCloudPtr& input_normals,
		const std::string model_type,
		const int max_iters,
		const float dist_thresh,
		const float rad_lim,
		const int min_seg_points,
		std::vector<pcl::PointIndices::Ptr>& inliers_vec,
		std::vector<pcl::ModelCoefficients::Ptr>& coefficients_vec,
		std::vector<PointCloudPtr>& seg_clouds){

  // Create the segmentation object and set all the parameters
  pcl::SACSegmentationFromNormals<PointT, Normal> seg;
  seg.setOptimizeCoefficients (true);
  if (model_type == "PLANE")
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  else if (model_type == "CYLINDER")
    seg.setModelType (pcl::SACMODEL_CYLINDER);

  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);  
  seg.setMaxIterations (max_iters);
  seg.setDistanceThreshold (dist_thresh);
  seg.setRadiusLimits (0, rad_lim);

  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<Normal> extract_normals;

  while (true) {
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    seg.setInputCloud (input_cloud);
    seg.setInputNormals (input_normals);
    seg.segment (*inliers, *coefficients);
    if(inliers->indices.size() < min_seg_points)
      break;
    
    inliers_vec.push_back(inliers);
    coefficients_vec.push_back(coefficients);

    PointCloudPtr cloud (new PointCloud);
    extract.setNegative(false);
    extract.setInputCloud(input_cloud);
    extract.setIndices(inliers);
    extract.filter(*cloud);
    seg_clouds.push_back(cloud);

    extract.setNegative (true);
    extract.setInputCloud (input_cloud);
    extract.setIndices (inliers);
    extract.filter (*input_cloud);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (input_normals);
    extract_normals.setIndices (inliers);
    extract_normals.filter (*input_normals);
  }

  return;
}

visualization_msgs::Marker process_cylin_cloud(const PointCloudPtr& input_cloud, const int id, const std::string frame_id, const pcl::ModelCoefficients::Ptr& coeff){

  PointCloudPtr projected_cloud (new PointCloud);
  pcl::ModelCoefficients::Ptr line (new pcl::ModelCoefficients);
  line->values.resize(6);
  for (unsigned int i = 0; i < 6; i++)
    line->values[i] = coeff->values[i];

  pcl::ProjectInliers<PointT> proj;
  proj.setModelType(pcl::SACMODEL_LINE);
  proj.setInputCloud(input_cloud);
  proj.setModelCoefficients(line);
  proj.filter(*projected_cloud);

  PointT minpt = PointT( 99, 99, 99);
  PointT maxpt = PointT(-99,-99,-99);
  for (unsigned int i = 0; i < projected_cloud->points.size(); i++){
    if (projected_cloud->points[i].x < minpt.x)
      minpt = projected_cloud->points[i];
    if (projected_cloud->points[i].x > maxpt.x)
      maxpt = projected_cloud->points[i];    
  }

  float ht = pcl::geometry::distance(minpt, maxpt);
  Eigen::Vector3f pos = Eigen::Vector3f((minpt.x+maxpt.x)/2, (minpt.y+maxpt.y)/2, (minpt.z+maxpt.z)/2);
  Eigen::Quaternionf quat = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(0,0,1),Eigen::Vector3f(coeff->values[3], coeff->values[4], coeff->values[5]));
  float dia = 2*coeff->values[6];

  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.id = id;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = pos[0];
  marker.pose.position.y = pos[1];
  marker.pose.position.z = pos[2];
  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.w();

  marker.scale.x = dia;
  marker.scale.y = dia;
  marker.scale.z = ht;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  marker.lifetime = ros::Duration();

  return marker;
}

jsk_recognition_msgs::BoundingBox process_plane_cloud(const PointCloudPtr& input_cloud, const int id, const std::string frame_id, const pcl::ModelCoefficients::Ptr& coeff){

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*input_cloud, centroid);

  PointCloudPtr projected_cloud (new PointCloud);
  pcl::ModelCoefficients::Ptr plane (new pcl::ModelCoefficients);
  plane->values.resize(4);
  for (unsigned int i = 0; i < 4; i++)
    plane->values[i] = coeff->values[i];
  
  pcl::ProjectInliers<PointT> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(input_cloud);
  proj.setModelCoefficients(plane);
  proj.filter(*projected_cloud);

  Eigen::Quaternionf quat = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(0,0,1),Eigen::Vector3f(coeff->values[0], coeff->values[1], coeff->values[2]));
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3,3>(0,0) = quat.normalized().toRotationMatrix();
  transform.block<3,1>(0,3) = centroid.head(3);
  Eigen::Matrix4f transform_inv = transform.inverse();
  pcl::transformPointCloud(*projected_cloud, *projected_cloud, transform_inv);

  Eigen::Vector4f minpt, maxpt;
  pcl::getMinMax3D(*projected_cloud, minpt, maxpt);
  Eigen::Vector3f dim = (maxpt - minpt).head(3);

  jsk_recognition_msgs::BoundingBox marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.label = id;

  marker.pose.position.x = transform(0,3);
  marker.pose.position.y = transform(1,3);
  marker.pose.position.z = transform(2,3);
  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.w();

  marker.dimensions.x = dim(0);
  marker.dimensions.y = dim(1);
  marker.dimensions.z = dim(2);

  return marker;
}
