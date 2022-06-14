#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointXYZ PointT;

// code taken from https://pcl.readthedocs.io/projects/tutorials/en/master/cylinder_segmentation.html#compiling-and-running-the-program
// need to refactor to fit my needs and see if this library is any good.
// If it doesn't do very well, I'll need to look into creating my own 
// version of ransac for cylinders/curves, can look at code from my RANSAC repo
// for the overarching layout. Key features of a cylinder is the axis, a point on the rectangle, and the radius

// Should remove the whole plane filtering thing out of here for now, it may eventually be useful for taking things out like leaves and such

// to visualize the result:
// run rviz with the plant_fitting.rviz file
// rosrun point_cloud_selector send_pcd_msg /new_pcd /home/christianforeman/catkin_ws/src/plant_fitting/pcs/horizontal_branch.pcd
// rosrun point_cloud_selector send_pcd_msg /new_cyl /home/christianforeman/catkin_ws/src/plant_fitting/pcs/cyl_comp.pcd
// rosrun point_cloud_selector send_pcd_msg /new_plane /home/christianforeman/catkin_ws/src/plant_fitting/pcs/plane_comp.pcd 

int main() {
  // All the objects needed
  pcl::PCDReader reader;
  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

  // Read in the cloud data
  reader.read ("pcs/branch_with_leaves.pcd", *cloud);
  std::cerr << "PointCloud has: " << cloud->size () << " data points." << std::endl;

//   // Build a passthrough filter to remove spurious NaNs and scene background
//   pass.setInputCloud (cloud);
//   pass.setFilterFieldName ("z");
//   pass.setFilterLimits (-5.0, 5.0);
//   pass.filter (*cloud_filtered);
//   std::cerr << "PointCloud after filtering has: " << cloud_filtered->size () << " data points." << std::endl;

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

//   // Create the segmentation object for the planar model and set all the parameters
//   seg.setOptimizeCoefficients (true);
//   seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
//   seg.setNormalDistanceWeight (0.1);
//   seg.setMethodType (pcl::SAC_RANSAC);
//   seg.setMaxIterations (100);
//   //seg.setDistanceThreshold (0.03);
//   seg.setDistanceThreshold(0.03);
//   seg.setInputCloud (cloud_filtered);
//   seg.setInputNormals (cloud_normals);
//   // Obtain the plane inliers and coefficients
//   seg.segment (*inliers_plane, *coefficients_plane);
//   std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

//   // Extract the planar inliers from the input cloud
//   extract.setInputCloud (cloud_filtered);
//   extract.setIndices (inliers_plane);
//   extract.setNegative (false);

//   // Write the planar inliers to disk
//   pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
//   extract.filter (*cloud_plane);
//   std::cerr << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;
//   writer.write ("pcs/plane_comp.pcd", *cloud_plane, false);

//   // Remove the planar inliers, extract the rest
//   extract.setNegative (true);
//   extract.filter (*cloud_filtered2);
//   extract_normals.setNegative (true);
//   extract_normals.setInputCloud (cloud_normals);
//   extract_normals.setIndices (inliers_plane);
//   extract_normals.filter (*cloud_normals2);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0, 0.1);
  seg.setInputCloud (cloud);
  seg.setInputNormals (cloud_normals);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;


  pcl::PointCloud<PointT>::Ptr pc_grid (new pcl::PointCloud<PointT>);

  reader.read ("pcs/pc_grid.pcd", *pc_grid);

  // Write the cylinder inliers to disk
  extract.setInputCloud (cloud);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_cylinder);
  if (cloud_cylinder->points.empty ()) 
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else
  {
	  std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->size () << " data points." << std::endl;
	  writer.write ("pcs/branch_with_leaves_cyl_comp.pcd", *cloud_cylinder, false);
  }
  return (0);
}