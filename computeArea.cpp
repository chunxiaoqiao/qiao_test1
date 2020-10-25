#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d_omp.h>//使用OMP需要添加的头文件
#include <pcl/common/pca.h>
using namespace pcl;
using namespace pcl::io;
using namespace std;
typedef pcl::PointXYZ PointT;
int main(int argc,char**argv)
{
	//加载点云数据
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud) == -1)
	{
		PCL_ERROR("Could not read file\n");
	}
    PLYWriter writer;
    writer.write("ply0.ply", cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(),true,true);
	PointT max_p,min_p;
	pcl::getMinMax3D(*cloud,min_p,max_p);
	cout<<min_p.x<<" "<<min_p.y<<" "<<min_p.z<<endl;
	cout<<max_p.x<<" "<<max_p.y<<" "<<max_p.z<<endl;
	cout<<"loaded point cloud"<<cloud->points.size()<<endl;
	for(int i=0;i<cloud->points.size();i++){
		cloud->points[i].z=0;	
	}
	pcl::visualization::CloudViewer viewer("cloud viewer");
    	viewer.showCloud(cloud);
    	//viewer.runOnVisualizationThreadOnce(viewerOneOff);
	pcl::ConvexHull<PointT> hull;                  
	hull.setInputCloud(cloud);                   
	hull.setDimension(2);
 	hull.setComputeAreaVolume(true);
	std::vector<pcl::Vertices> polygons;                 
	pcl::PointCloud<PointT>::Ptr surface_hull(new pcl::PointCloud<PointT>);
	hull.reconstruct(*surface_hull, polygons);   
	pcl::io::savePCDFileASCII("../plane.pcd",*cloud);
	pcl::io::savePCDFileASCII("../hull.pcd",*surface_hull);
	cout<<surface_hull->points.size()<<endl;
	double area= hull.getTotalArea();  
	cout<<area<<endl;
	return 0;
}

