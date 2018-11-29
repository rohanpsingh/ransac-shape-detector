#include "common_headers.h"
#include "cloud_utils.h"

float max_dist;
bool debug_enable;
int plane_min_points;
int cylinder_min_points;
float voxel_leaf_size;

ros::Publisher point_pub;
ros::Publisher marker_pub_cylin;
ros::Publisher marker_pub_plane;



void process_data(const PointCloudPtr& input_cloud){

  PointCloudPtr read_cloud = input_cloud;
  PointCloudPtr cloud_filt = cloud_downsample(read_cloud, voxel_leaf_size);
  PointCloudPtr scene_cloud = cloud_passthrough(cloud_filt, "z", 0, max_dist);

  // Estimate normals for scene_cloud  
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  NormalCloudPtr scene_normals (new NormalCloud);
  pcl::NormalEstimation<PointT, Normal> ne;
  ne.setSearchMethod(tree);
  ne.setInputCloud (scene_cloud);
  ne.setKSearch(50);
  ne.compute (*scene_normals);

  //extract as many planes as possible
  std::vector<pcl::ModelCoefficients::Ptr> coefficients_plane;
  std::vector<pcl::PointIndices::Ptr> inliers_plane;
  std::vector<PointCloudPtr> plane_clouds;
  seg_ransac(scene_cloud, scene_normals, "PLANE", 100, 0.03, 0, plane_min_points, inliers_plane, coefficients_plane, plane_clouds);
  
  //extract as many cylinders as possible
  std::vector<pcl::ModelCoefficients::Ptr> coefficients_cylin;
  std::vector<pcl::PointIndices::Ptr> inliers_cylin;
  std::vector<PointCloudPtr> cylin_clouds;
  seg_ransac(scene_cloud, scene_normals, "CYLINDER", 10000, 0.05, 0.1, cylinder_min_points, inliers_cylin, coefficients_cylin, cylin_clouds);

  sensor_msgs::PointCloud2 pub_msg;
  visualization_msgs::MarkerArray cylinarray;
  for (unsigned int i = 0; i < cylin_clouds.size(); i++){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = cylin_clouds[i];
    pcl::ModelCoefficients::Ptr coeff = coefficients_cylin[i];
    visualization_msgs::Marker marker = process_cylin_cloud(cloud, i, read_cloud->header.frame_id, coeff);
    cylinarray.markers.push_back(marker);
  }
  marker_pub_cylin.publish(cylinarray);

  jsk_recognition_msgs::BoundingBoxArray planearray;
  for (unsigned int i = 0; i < plane_clouds.size(); i++){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = plane_clouds[i];
    pcl::ModelCoefficients::Ptr coeff = coefficients_plane[i];
    jsk_recognition_msgs::BoundingBox marker = process_plane_cloud(cloud, i, read_cloud->header.frame_id, coeff);
    planearray.boxes.push_back(marker);
  }
  planearray.header.frame_id = read_cloud->header.frame_id;
  planearray.header.stamp = ros::Time::now();
  marker_pub_plane.publish(planearray);

  PointCloudPtr test_cloud = plane_clouds[1];
  pcl::toROSMsg(*test_cloud,pub_msg);
  pub_msg.header.frame_id = read_cloud->header.frame_id;
  pub_msg.header.stamp = ros::Time::now();
  point_pub.publish(pub_msg);

  return;  
}


void msgCallback(const sensor_msgs::PointCloud2ConstPtr& pcd){

  pcl::ScopeTime t("msgCallback");
  
  // Load scene
  PointCloudPtr read_cloud (new PointCloud);
  pcl::fromROSMsg(*pcd, *read_cloud);
  if (read_cloud->points.empty()){
    ROS_ERROR("Empty point cloud received!");
    return;
  }

  process_data(read_cloud);  
  
  return;
}


int main(int argc, char **argv){

    ros::init(argc, argv, "shape_detect");
    ros::NodeHandle nh("~");

    nh.param("max_distance", max_dist, float(2));
    nh.param("plane_min_points", plane_min_points, int(2000));
    nh.param("cylinder_min_points", cylinder_min_points, int(500));
    nh.param("debugging", debug_enable, bool(true));
    nh.param("voxel_leaf_size", voxel_leaf_size, float(0.01));    
    //    nh.param("pub_frame_id", frame_id, std::string("camera_depth_optical_frame"));

    if (debug_enable) {
      std::cout << "Printing out rosparam vals..." << std::endl;
      std::cout << "Debugging enabled." << std::endl;
      std::cout << "max_distance: " << max_dist << std::endl;
      std::cout << "plane_min_point: " << plane_min_points << std::endl;
      std::cout << "cylinder_min_points: " << cylinder_min_points << std::endl;
      std::cout << "============DONE============" << std::endl;
    }

    ros::Subscriber sub = nh.subscribe("input_cloud",1, msgCallback);
    point_pub = nh.advertise<sensor_msgs::PointCloud2>("output_cloud",1);
    marker_pub_cylin = nh.advertise<visualization_msgs::MarkerArray>("output_marker_cylin",1);
    marker_pub_plane = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("output_marker_plane",1);    
    
    ros::spin();
  
    return 0;
}
