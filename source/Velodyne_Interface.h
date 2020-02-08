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

//#include <pcl/common/common_headers.h>

//#include <pcl/filters/passthrough.h>
#pragma comment(lib,"opengl32.lib")	
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);

#include"TimeString.h"

#define M_PI 3.14159265359

using namespace std;
typedef pcl::PointXYZI PointType;

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

	class CRobotState {
	public:
		double x;
		double y;
		double z;
	};


	bool connect(string ipaddress, string port);
	bool disconnect();
	void initVisualizer();
	void ToggleWrite();
	void execute(int nPeriod);
	void getPC_oneframe_cb(const pcl::PointCloud<PointType>::ConstPtr&);
	void Process();
	void show();
	void ShowPcdFile(pcl::PointCloud<PointType>::Ptr p_cloud);
	void ReadAndShowOne(string filename_arg);
	void TimerWrite();
	pcl::PointCloud<PointType>::Ptr ElevationAngleFiltering(pcl::PointCloud<PointType>::Ptr cloud);
	void show_2D();
	void ReadAndShow2Clouds(string filename1, string filename2);

	//bool start_thread(void (*funcPtr)(void*));	//CProcessThread::startを参考にすべし？
	//static CVelodyneInterface* getInstance();
	//void releaseInstance();

	// Point Cloud Color Handler
	pcl::visualization::PointCloudColorHandler<PointType>::Ptr m_handler;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;				//大丈夫？

	CVelodyneInterface() {
	}

	//CVelodyneInterface(boost::mutex &mutex) : m_mutex(mutex) {}

private:
	//これはProcessを使う場合は不要．
	//boost::mutex &m_mutex;			//これは参照受け取り？よく分からない．
	boost::mutex m_mutex;
	boost::shared_ptr<pcl::VLPGrabber> grabber;
	boost::signals2::connection connection;
	boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> function;

public:
	pcl::PointCloud<PointType>::ConstPtr getPointCloud_ConstPtr();
	pcl::PointCloud<PointType>::Ptr getPointCloud();
	pcl::PointCloud<PointType>::Ptr get2DPointCloud();
	pcl::PointCloud<PointType>::Ptr get2DPointCloud_(boost::mutex &mutex_arg);

	double* convertPCto181DegArray(pcl::PointCloud<PointType>::Ptr);
	
	vector<double> getDegVector(pcl::PointCloud<PointType>::Ptr cloud, int NumOfData);

	void all(string ipaddress = "", string port = "");


	//
	void getOnlyShow(double*);
	void getOnlyShow2(vector<double>);


	void getOnlyShow3(vector<CRobotState> state_arg);
	//vector<CRobotState>* getDegStateVector(pcl::PointCloud<PointType>::Ptr cloud, int NumOfData);

	//vector<CRobotState> getDegStateVector(pcl::PointCloud<PointType>::Ptr cloud, int NumOfData);

	vector<CRobotState> CVelodyneInterface::getDegStateVector(pcl::PointCloud<PointType>::Ptr cloud, int NumOfData) {
		//vector<CRobotState::Position> PositionVector;
		vector<CRobotState> StateVector;
		//vector<double> RangeVector;
		vector<double> MinErrorArray;
		//auto Itr = RangeVector_181.begin();

		//debug
		CTimeString ts;
		cout << "start at " << ts.getTimeString() << endl;

		double resolution = 180. / (NumOfData - 1);//resolution		181->1deg, 361->0.5deg 



												   //initialize
		for (int i = 0; i < NumOfData; i++) {
			CRobotState state;
			state.x = 100.*cos((i*(180. / (NumOfData - 1)) - 90)*M_PI / 180.);
			state.y = 100.*sin((i*(180. / (NumOfData - 1)) - 90)*M_PI / 180.);
			state.z = 0.;

			StateVector.push_back(state);

			//position_.x = 100.;
			//position_.y = 0.;
			//position_x = 0.;
			//PositionVector.push_back(position_)

			//RangeVector.push_back(100.);		//Max Range
			MinErrorArray.push_back(resolution);
		}

		//calculate reduced data
		for (int i = 0; i < cloud->size(); i++) {
			double angle_deg;
			double standard;		// 1.0, 2.0, 3.0, ...(integer)
			double error;		//distance from angle to standard

			angle_deg = atan2(cloud->points[i].x, cloud->points[i].y)*180. / M_PI;

			//debug
			// convertion from clockwise to anticlockwise
			angle_deg = -angle_deg + 90.;
			if (angle_deg >= 180.) angle_deg -= 360.;

			//debug
			//cout << "range["<<angle_deg<<"] =" << sqrt(pow(cloud->points[i].x, 2.) + pow(cloud->points[i].y, 2.)) << endl;

			if (!(-90. - resolution / 2. < angle_deg && angle_deg < 90. + resolution / 2.)) continue;


			int standard_num;		//-180,-179,...,0,...,180
			if (angle_deg >= 0) standard_num = (int)((angle_deg + resolution / 2.) / resolution);
			else standard_num = (int)((angle_deg - resolution / 2.) / resolution);


			standard = (double)(standard_num)*resolution;		//-90,-89.5,...,,89.5,90
																//このstandardとは，最も近い「キリの良い値」である．
																//キリの良い値=resolutionをk倍したもの

			error = fabs(angle_deg - standard);

			//debug
			//cout << "array_num = " << standard_num << endl;

			if (MinErrorArray[standard_num + (NumOfData - 1) / 2] > error) {
				MinErrorArray[standard_num + (NumOfData - 1) / 2] = error;

				//そのまま代入して大丈夫？座標変換とかが心配．
				//PositionVector[standard_num + (NumOfData - 1) / 2].x = cloud->points[i].x;
				//PositionVector[standard_num + (NumOfData - 1) / 2].y = cloud->points[i].y;

				//RangeVector[standard_num + (NumOfData - 1) / 2] = sqrt(pow(cloud->points[i].x, 2.) + pow(cloud->points[i].y, 2.));
				StateVector[standard_num + (NumOfData - 1) / 2].x = cloud->points[i].x;
				StateVector[standard_num + (NumOfData - 1) / 2].y = cloud->points[i].y;

			}
		}

		cout << "end at " << ts.getTimeString() << endl;

		//return RangeVector;
		return StateVector;

	}

	void ShowOnlySequent(string foldername_);

	void HandRegistration(string foldername_);

	Eigen::Matrix4d calcHomogeneousMatrixFromVector6d(double X_, double Y_, double Z_,
		double Roll_, double Pitch_, double Yaw_);

	Eigen::Vector6d calcVector6dFromHomogeneousMatrix(Eigen::Matrix4d transformation_Node);

	Eigen::Affine3f calcAffine3fFromHomogeneousMatrix(Eigen::Matrix4d input_Mat);


};