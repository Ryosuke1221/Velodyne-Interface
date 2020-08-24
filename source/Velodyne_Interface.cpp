#include"Velodyne_Interface.h"

void CVelodyneInterface::all(string ipaddress, string port)
{
	int WhichProcess = 0;
	string filename1, filename2;
	bool b_finish = false;

	enum OPTION {
		EN_escape = 0,
		EN_FreeSpace,
		EN_ToggleWrite,
		EN_capture,
		EN_capture2D,
		EN_sequentshow,
	};

	while (!b_finish)
	{
		cout << endl;
		cout << "please input process number" << endl;
		cout << EN_escape << ": escape" << endl;
		cout << EN_FreeSpace << ": free space" << endl;
		cout << EN_ToggleWrite << ": ToggleWrite" << endl;
		cout << EN_capture << ": capture and show" << endl;
		cout << EN_capture2D << ": capture and show in 2D" << endl;
		cout << EN_sequentshow << ": sequent show" << endl;

		cin >> WhichProcess;
		switch (WhichProcess) {
		case EN_escape:
			//escape
			b_finish = true;
			break;

		case EN_FreeSpace:
			FreeSpace();
			break;

		case EN_ToggleWrite:
			initVisualizer();
			connect(ipaddress, port);
			ToggleWrite("../data/00_save");
			disconnect();
			break;

		case EN_capture:
			initVisualizer();
			connect(ipaddress, port);
			showPointCloud_realtime();
			disconnect();
			break;

		case EN_capture2D:
			//capture and show in 2D
			initVisualizer();
			connect(ipaddress, port);
			showPointCloud_realtime_2D();
			disconnect();
			break;

		case EN_sequentshow:
			//sequent show
			initVisualizer();
			showPointCloud_sequently("../data");
			break;
		}

	}
	
	cout << "process finished (press:ESC)" << endl;
	GetAsyncKeyState(VK_ESCAPE);
	while (1)
		if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) break;

}

Eigen::Matrix4d CVelodyneInterface::calcHomogeneousMatrixFromVector6d(double X_, double Y_, double Z_,
	double Roll_, double Pitch_, double Yaw_) {
	Eigen::Matrix4d	transformation_Position = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T_mat = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d Roll_mat = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d Pitch_mat = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d Yaw_mat = Eigen::Matrix4d::Identity();
	T_mat(0, 3) = X_;
	T_mat(1, 3) = Y_;
	T_mat(2, 3) = Z_;
	Roll_mat(1, 1) = cos(Roll_);
	Roll_mat(1, 2) = -sin(Roll_);
	Roll_mat(2, 1) = sin(Roll_);
	Roll_mat(2, 2) = cos(Roll_);
	Pitch_mat(0, 0) = cos(Pitch_);
	Pitch_mat(2, 0) = -sin(Pitch_);
	Pitch_mat(0, 2) = sin(Pitch_);
	Pitch_mat(2, 2) = cos(Pitch_);
	Yaw_mat(0, 0) = cos(Yaw_);
	Yaw_mat(0, 1) = -sin(Yaw_);
	Yaw_mat(1, 0) = sin(Yaw_);
	Yaw_mat(1, 1) = cos(Yaw_);
	transformation_Position = T_mat * Yaw_mat * Pitch_mat * Roll_mat;
	return transformation_Position;
}

Eigen::Vector6d CVelodyneInterface::calcVector6dFromHomogeneousMatrix(Eigen::Matrix4d transformation_Node) {
	Eigen::Vector6d XYZRPY = Eigen::Vector6d::Zero();
	double X_, Y_, Z_, Roll_, Pitch_, Yaw_;
	X_ = transformation_Node(0, 3);
	Y_ = transformation_Node(1, 3);
	Z_ = transformation_Node(2, 3);
	if (transformation_Node(2, 0) == -1.) {
		Pitch_ = M_PI / 2.0;
		Roll_ = 0.;
		Yaw_ = atan2(transformation_Node(1, 2), transformation_Node(1, 1));
	}
	else if (transformation_Node(2, 0) == 1.) {
		Pitch_ = -M_PI / 2.0;
		Roll_ = 0.;
		Yaw_ = atan2(-transformation_Node(1, 2), transformation_Node(1, 1));
	}
	else {
		Yaw_ = atan2(transformation_Node(1, 0), transformation_Node(0, 0));
		Roll_ = atan2(transformation_Node(2, 1), transformation_Node(2, 2));
		double cos_Pitch;
		if (cos(Yaw_) == 0.) {
			cos_Pitch = transformation_Node(0, 0) / sin(Yaw_);
		}
		else 	cos_Pitch = transformation_Node(0, 0) / cos(Yaw_);

		Pitch_ = atan2(-transformation_Node(2, 0), cos_Pitch);
	}
	if (!(-M_PI < Roll_)) Roll_ += M_PI;
	else if (!(Roll_ < M_PI)) Roll_ -= M_PI;
	if (!(-M_PI < Pitch_)) Pitch_ += M_PI;
	else if (!(Pitch_ < M_PI)) Pitch_ -= M_PI;
	if (!(-M_PI < Yaw_)) Yaw_ += M_PI;
	else if (!(Yaw_ < M_PI)) Yaw_ -= M_PI;

	XYZRPY << X_, Y_, Z_,
		Roll_, Pitch_, Yaw_;
	//cout << "Roll_ = " << Roll_ << endl;
	//cout << "Pitch_ = " << Pitch_ << endl;
	//cout << "Yaw_ = " << Yaw_ << endl;

	return XYZRPY;
}

Eigen::Affine3f CVelodyneInterface::calcAffine3fFromHomogeneousMatrix(Eigen::Matrix4d input_Mat) {
	Eigen::Affine3f Trans_Affine = Eigen::Affine3f::Identity();
	Eigen::Vector6d Trans_Vec = Eigen::Vector6d::Identity();
	Trans_Vec = calcVector6dFromHomogeneousMatrix(input_Mat);
	Trans_Affine.translation() << Trans_Vec(0, 0), Trans_Vec(1, 0), Trans_Vec(2, 0);
	Trans_Affine.rotate(Eigen::AngleAxisf(Trans_Vec(5, 0), Eigen::Vector3f::UnitZ()));
	Trans_Affine.rotate(Eigen::AngleAxisf(Trans_Vec(4, 0), Eigen::Vector3f::UnitY()));
	Trans_Affine.rotate(Eigen::AngleAxisf(Trans_Vec(3, 0), Eigen::Vector3f::UnitX()));
	return Trans_Affine;
}

void CVelodyneInterface::FreeSpace()
{

}

void CVelodyneInterface::showPointCloud_sequently(string dir_)
{

	string s_folder;
	{
		vector<string> filenames_folder;

		CTimeString::getFileNames_folder(dir_, filenames_folder);
		for (int i = 0; i < filenames_folder.size(); i++)
		{
			string s_i = to_string(i);
			if (s_i.size() < 2) s_i = " " + s_i;
			cout << "i:" << s_i << " " << filenames_folder[i] << endl;
		}
		cout << endl;
		cout << "input folder(index) you want to calc ->";
		int i_folder;
		cin >> i_folder;
		s_folder = filenames_folder[i_folder];
	}

	//input pointcloud
	vector<pcl::PointCloud<PointType>::Ptr> cloud_vec;
	{
		vector<string> filenames_;
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_, ".pcd");
		for (int i = 0; i < filenames_.size(); i++)
		{
			pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
			pcl::io::loadPCDFile(dir_ + "/" + s_folder + "/" + filenames_[i], *cloud);
			cloud->is_dense = true;
			cloud_vec.push_back(cloud);
		}
	}

	cout << "press SPACE to go next" << endl;
	cout << "press ESC to finish" << endl;

	int index_cloud = 0;
	pcl::PointCloud<PointType>::Ptr cloud_showing(new pcl::PointCloud<PointType>());

	while (!m_viewer->wasStopped())
	{

		//change PointCloud
		if (((GetAsyncKeyState(VK_SPACE) & 1) == 1) && (index_cloud != cloud_vec.size()))
		{

			pcl::copyPointCloud(*cloud_vec[index_cloud], *cloud_showing);
			cout << "index:" << index_cloud << endl;
			index_cloud++;
		}

		//go to finish
		if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) break;

		//show point cloud
		ShowPcdFile(cloud_showing);

	}

	cout << "finished" << endl;
	m_viewer->close();
}

void CVelodyneInterface::showPointCloud_realtime_2D()
{

	pcl::PointCloud<PointType>::Ptr cloud;
	//pcl::PointCloud<PointType>::ConstPtr cloud;

	while (!m_viewer->wasStopped()) {
		// Update Viewer
		m_viewer->spinOnce();

		cloud = get2DPointCloud();					//中にm_mutexが入っているので，同時アクセスの危険は薄いと思う．

		if (cloud) {
			// Update Point Cloud
			m_handler->setInputCloud(cloud);
			if (!m_viewer->updatePointCloud(cloud, *m_handler, "cloud")) {
				m_viewer->addPointCloud(cloud, *m_handler, "cloud");
			}
		}

		if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) break;

	}

	cout << "finished" << endl;
	m_viewer->close();
}

pcl::PointCloud<PointType>::Ptr CVelodyneInterface::getPointCloud() {
	boost::mutex::scoped_lock lock(m_mutex);
	pcl::PointCloud<PointType>::Ptr PointCloud(new pcl::PointCloud<PointType>(*m_PointCloud_ConstPtr));	//ConstPtr -> Ptr
	return PointCloud;

}

pcl::PointCloud<PointType>::Ptr CVelodyneInterface::get2DPointCloud()
{
	boost::mutex::scoped_lock lock(m_mutex);
	pcl::PointCloud<PointType>::Ptr cloud_ptr(new pcl::PointCloud<PointType>(*m_PointCloud_ConstPtr));	//ConstPtr -> Ptr
	float range_pitch_rad = 30. * D2R;
	int num_laser = 16;
	return ElevationAngleFiltering(cloud_ptr, range_pitch_rad, num_laser);
}

pcl::PointCloud<PointType>::ConstPtr CVelodyneInterface::getPointCloud_ConstPtr()
{
	boost::mutex::scoped_lock lock(m_mutex);
	return m_PointCloud_ConstPtr;
}

pcl::PointCloud<PointType>::Ptr CVelodyneInterface::ElevationAngleFiltering(pcl::PointCloud<PointType>::Ptr cloud,
	float range_pitch_rad, int num_laser)
{
	float range_pitch_divide = range_pitch_rad / (float)(num_laser - 1);
	float th_pitch_abs = (range_pitch_divide + range_pitch_divide * 2) / 2.;	//mean of most vertical and second vertical
	pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>());
	for (size_t i = 0; i < cloud->points.size(); i++) {
		double sin_elevation = cloud->points[i].z / sqrt(pow(cloud->points[i].x, 2.) + pow(cloud->points[i].y, 2.) + pow(cloud->points[i].z, 2.));
		double angle_abs = fabs(asin(sin_elevation));
		if (angle_abs >= th_pitch_abs) continue;
		cloud->points[i].z = 0.;
		cloud_filtered->points.push_back(cloud->points[i]);
	}
	//process to prevent error occuring
	cloud_filtered->width = 1;
	cloud_filtered->height = cloud_filtered->points.size();
	cloud_filtered->is_dense = true;
	return cloud_filtered;
}

void CVelodyneInterface::ShowPcdFile(pcl::PointCloud<PointType>::Ptr p_cloud)
{

	m_viewer->spinOnce();

	if (p_cloud) {

		// Update Point Cloud
		m_handler->setInputCloud(p_cloud);
		if (!m_viewer->updatePointCloud(p_cloud, *m_handler, "cloud")) {
			m_viewer->addPointCloud(p_cloud, *m_handler, "cloud");
			cout << "succeeded showing" << endl;
		}

	}
	else cout << "fail to show" << endl;
}

void CVelodyneInterface::ToggleWrite(string dir_) {

	bool b_AttemptCapture = false;

	int num_data = 0;

	cout << "Press SPACE to screenshot" << endl;
	cout << "Press SPACE to finish" << endl;

	while (!m_viewer->wasStopped()) {
		// Update Viewer
		m_viewer->spinOnce();

		if ((GetAsyncKeyState(VK_SPACE) & 1) == 1) {
			b_AttemptCapture = true;
			cout << "toggled!" << endl;
		}

		boost::mutex::scoped_try_lock lock(m_mutex);

		if (lock.owns_lock() && m_PointCloud_ConstPtr) {
			// Update Point Cloud
			m_handler->setInputCloud(m_PointCloud_ConstPtr);
			if (!m_viewer->updatePointCloud(m_PointCloud_ConstPtr, *m_handler, "cloud")) {
				m_viewer->addPointCloud(m_PointCloud_ConstPtr, *m_handler, "cloud");
			}

			if (b_AttemptCapture == true) {
				string filename_;
				filename_ = CTimeString::getTimeString() + ".pcd";
				cout << filename_ << endl;
				pcl::io::savePCDFile<PointType>(dir_ + "/" + filename_, *m_PointCloud_ConstPtr);
				b_AttemptCapture = false;
				cout << "↑Written!" << endl;
				num_data++;

			}
		}

		if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) break;
	}

	cout << "Finished!" << endl;
	m_viewer->close();
}

void CVelodyneInterface::Process() {

	initVisualizer();
	
	//connect();

	//描画をする関数
	showPointCloud_realtime();

	disconnect();

}

void CVelodyneInterface::showPointCloud_realtime() {

	while (!m_viewer->wasStopped()) {
		// Update Viewer
		m_viewer->spinOnce();

		boost::mutex::scoped_try_lock lock(m_mutex);
		//m_PointCloud_ConstPtr = getPointCloud();
		if (lock.owns_lock() && m_PointCloud_ConstPtr) {
			// Update Point Cloud
			m_handler->setInputCloud(m_PointCloud_ConstPtr);
			if (!m_viewer->updatePointCloud(m_PointCloud_ConstPtr, *m_handler, "cloud")) {
				m_viewer->addPointCloud(m_PointCloud_ConstPtr, *m_handler, "cloud");
			}
		}

		if ((GetAsyncKeyState(VK_SPACE) & 1) == 1) break;

	}

	cout << "toggled!" << endl;
	m_viewer->close();

}

void CVelodyneInterface::initVisualizer()
{

	// PCL Visualizer
	m_viewer.reset(new pcl::visualization::PCLVisualizer("Velodyne Viewer"));

	m_viewer->addCoordinateSystem(3.0, "coordinate");
	m_viewer->setBackgroundColor(0.0, 0.0, 0.0, 0);
	m_viewer->initCameraParameters();
	m_viewer->setCameraPosition(0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
	//m_viewer->removeCoordinateSystem("coordinate");		//remove axis in viewer

	// Point Cloud Color Handler
	//pcl::visualization::PointCloudColorHandler<PointType>::Ptr handler;	//これはメンバ変数にした．
	const std::type_info& type = typeid(PointType);
	if (type == typeid(pcl::PointXYZ)) {
		std::vector<double> color = { 255.0, 255.0, 255.0 };
		boost::shared_ptr<pcl::visualization::PointCloudColorHandlerCustom<PointType>> color_handler(new pcl::visualization::PointCloudColorHandlerCustom<PointType>(color[0], color[1], color[2]));
		m_handler = color_handler;
	}
	else if (type == typeid(pcl::PointXYZI)) {
		boost::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<PointType>> color_handler(new pcl::visualization::PointCloudColorHandlerGenericField<PointType>("intensity"));
		m_handler = color_handler;
	}
	else if (type == typeid(pcl::PointXYZRGBA)) {
		boost::shared_ptr<pcl::visualization::PointCloudColorHandlerRGBField<PointType>> color_handler(new pcl::visualization::PointCloudColorHandlerRGBField<PointType>());
		m_handler = color_handler;
	}
	else if (type == typeid(pcl::PointXYZRGB)) {
		boost::shared_ptr<pcl::visualization::PointCloudColorHandlerRGBField<PointType>> color_handler(new pcl::visualization::PointCloudColorHandlerRGBField<PointType>());
		m_handler = color_handler;
	}
	else {
		throw std::runtime_error("This PointType is unsupported.");
	}


}

//センサデータ取得のスレッドを走らせるための関数
void CVelodyneInterface::execute(int nPeriod) 
{

	//関数オブジェクトの生成？
	function =
		[this](const pcl::PointCloud<PointType>::ConstPtr& ptr) {

		//boost::mutex::scoped_lock lock(m_mutex);

		/* Point Cloud Processing */
		getPC_oneframe_cb(ptr);

	};

	connection = grabber->registerCallback(function);

}

bool CVelodyneInterface::connect(string ipaddress, string port)
{
	//init sensor status

	//arg: ipaddress, port

	std::cout << "-ipadress : " << ipaddress << std::endl;
	std::cout << "-port : " << port << std::endl;

	if (!ipaddress.empty()) {
		cout << "Capture from Sensor..." << std::endl;
		grabber = boost::shared_ptr<pcl::VLPGrabber>(new pcl::VLPGrabber(boost::asio::ip::address::from_string(ipaddress), boost::lexical_cast<unsigned short>(port)));
	}
	else {
		cout << "pcap and ipaddress are not found" << endl;
		return false;
	}

	first_success_b = false;

	//thread start
	this->execute(1);

	grabber->start();
}

//boolの方がいい？
bool CVelodyneInterface::disconnect()
{

	//センサデータ取得のループを終わらせる処理と，スレッドを終わらせる処理を書く．
	grabber->stop();

	if (connection.connected()) {
		connection.disconnect();
		cout << "disconnected" << endl;
	}
	else return false;
}

//センサデータを1フレーム分取得する，繰り返されるべき関数
void CVelodyneInterface::getPC_oneframe_cb(const pcl::PointCloud<PointType>::ConstPtr& ptr) 
{	//call back?

	boost::mutex::scoped_lock lock(m_mutex);
	m_PointCloud_ConstPtr = ptr;

	if (m_PointCloud_ConstPtr && first_success_b == false) {
		first_success_b = true;
		cout << "succeeded connecting!!" << endl;

	}

}



