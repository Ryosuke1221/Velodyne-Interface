#pragma once
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/console/parse.h>
#include <vector>

#include<pcl/registration/transforms.h>

#include <pcl/filters/approximate_voxel_grid.h>

//#include <pcl/common/common_headers.h>

//#include <pcl/filters/passthrough.h>
#pragma comment(lib,"opengl32.lib")	
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);

#include"TimeString.h"

#define M_PI 3.14159265359
#define D2R 0.017453288888889
#define R2D 57.29579143313326

using namespace std;
typedef pcl::PointXYZI PointType;
//typedef pcl::PointXYZRGB PointType;

namespace Eigen {

	/// Extending Eigen namespace by adding frequently used matrix type
	typedef Eigen::Matrix<double, 6, 6> Matrix6d;
	typedef Eigen::Matrix<double, 6, 1> Vector6d;

}    // namespace Eigen


class CVelodyneInterface
{
private:
	//output data
	pcl::PointCloud<PointType>::ConstPtr m_PointCloud_ConstPtr;   //[m] scale
	bool first_success_b;

public:

	bool connect(string ipaddress, string port);
	bool disconnect();
	void initVisualizer();
	void ToggleWrite(string dir_);
	void execute(int nPeriod);
	void getPC_oneframe_cb(const pcl::PointCloud<PointType>::ConstPtr&);
	void Process();
	void showPointCloud_realtime();
	void showPointCloud_realtime_2D();
	void ShowPcdFile(pcl::PointCloud<PointType>::Ptr p_cloud);

	pcl::PointCloud<PointType>::Ptr ElevationAngleFiltering(pcl::PointCloud<PointType>::Ptr cloud, float range_pitch_rad, int num_laser);

	// Point Cloud Color Handler
	pcl::visualization::PointCloudColorHandler<PointType>::Ptr m_handler;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;

	CVelodyneInterface() {
	}

private:
	boost::mutex m_mutex;
	boost::shared_ptr<pcl::VLPGrabber> grabber;
	boost::signals2::connection connection;
	boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> function;

public:
	pcl::PointCloud<PointType>::ConstPtr getPointCloud_ConstPtr();
	pcl::PointCloud<PointType>::Ptr getPointCloud();
	pcl::PointCloud<PointType>::Ptr get2DPointCloud();
	
	void all(string ipaddress = "", string port = "");

	void showPointCloud_sequently(string foldername_);

	Eigen::Matrix4d calcHomogeneousMatrixFromVector6d(double X_, double Y_, double Z_,
		double Roll_, double Pitch_, double Yaw_);

	Eigen::Vector6d calcVector6dFromHomogeneousMatrix(Eigen::Matrix4d transformation_Node);

	Eigen::Affine3f calcAffine3fFromHomogeneousMatrix(Eigen::Matrix4d input_Mat);

	void FreeSpace();

};