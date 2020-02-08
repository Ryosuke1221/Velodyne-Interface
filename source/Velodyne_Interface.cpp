#include"Velodyne_Interface.h"

void CVelodyneInterface::all(string ipaddress, string port)
{
	int WhichProcess = 0;
	string filename1, filename2;

	enum OPTION {

		EN_escape = 1,
		EN_ReadAndShowOne,
		EN_show2PointClouds,
		EN_ToggleWrite,
		EN_TimerWrite,
		EN_capture,
		EN_capture2D,
		EN_sequentshow,
		EN_handregistration

	};

	cout << "please input process number" << endl;
	cout << EN_escape << ": escape" << endl;
	cout << EN_ReadAndShowOne << ": ReadAndShowOne" << endl;
	cout << EN_show2PointClouds << ": show 2 PointClouds" << endl;
	cout << EN_ToggleWrite << ": ToggleWrite" << endl;
	cout << EN_TimerWrite << ": TimerWrite" << endl;
	cout << EN_capture << ": capture and show" << endl;
	cout << EN_capture2D << ": capture and show in 2D" << endl;
	cout << EN_sequentshow << ": sequent show" << endl;
	cout << EN_handregistration << ": hand registration" << endl;

	cin >> WhichProcess;
	switch (WhichProcess) {
	case EN_escape:
		//escape
		break;

	case EN_ReadAndShowOne:
		//ReadAndShowOne
		initVisualizer();
		//velodyne.ReadAndShowOne("\../savedfolder/20181017_0242_45_080.pcd");
		//velodyne.ReadAndShowOne("\../savedfolder/20181017_0243_07_597.pcd");
		//ReadAndShowOne("\../savedfolder/20190829_1434_16_859.pcd");			//for making it be plate
		ReadAndShowOne("\../savedfolder/20200119_1101_51_127.pcd");			//for making it be plate

																							//20181017_0242_56_345
																							//20181017_0243_07_597
																							//連続で読み込む場合は，ファイルを最初にまとめて読み込むべし．読み込む用の関数が要る．
																							//ウィンドウを閉じる処理を入れるべき．
		break;

	case EN_show2PointClouds:
		//show 2 PointClouds
		//string filename1;
		filename1 = "\../savedfolder/20181017_0242_45_080.pcd";
		//string filename2;
		filename2 = "\../savedfolder/20181017_0243_07_597.pcd";
		ReadAndShow2Clouds(filename1, filename2);
		break;


	case EN_ToggleWrite:
		//ToggleWrite
		initVisualizer();
		connect(ipaddress, port);
		ToggleWrite();
		disconnect();
		break;

	case EN_TimerWrite:
		//TimerWrite
		connect(ipaddress, port);
		TimerWrite();
		disconnect();
		break;

	case EN_capture:
		//capture and show
		initVisualizer();
		connect(ipaddress, port);
		show();
		disconnect();
		break;

	case EN_capture2D:
		//capture and show in 2D
		initVisualizer();
		connect(ipaddress, port);
		show_2D();
		disconnect();
		break;

	case EN_sequentshow:
		//sequent show
		initVisualizer();
		//ReadAndShowOne("\../savedfolder/20190823_1543_22_646.pcd");			//for making it be plate

		//ShowOnlySquent();
		//ShowOnlySquent("\../savedfolder/sequent/", 14);
		//ShowOnlySequent("\../savedfolder/sequent/");
		ShowOnlySequent("\../savedfolder/naraha summer/sequent/");
		//ShowOnlySequent("\../savedfolder/20200119/PointCloud/");
		break;

	case EN_handregistration:
		initVisualizer();
		HandRegistration("\../savedfolder/naraha summer/sequent/",0);
		break;

	}

	getchar();

}

void CVelodyneInterface::HandRegistration(string foldername_,int num_start) {

	cout << "HandRegistration started!" << endl;
	Sleep(1 * 1000);

	std::string filename_;
	//input txt name
	filename_ = foldername_ + "tramsformation.txt";

	pcl::PointCloud<PointType>::Ptr cloud_show(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr cloud_show_static(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr cloud_moving(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr cloud_moving_before(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());

	CTimeString time_;

	Eigen::Affine3f Trans_;

	double disp_translation = 0.;
	double disp_rotation = 0.;
	disp_translation = 0.05;
	disp_rotation = 0.5 * M_PI / 180.;

	int num_PC_now = num_start;

	bool b_makeNewPC = true;
	bool b_first = true;
	bool b_escaped = false;

	enum KEYNUM {
		NONE ,
		UP ,
		DOWN ,
		RIGHT ,
		LEFT ,
		TURN_R ,
		TURN_L ,
		ENTER ,
		SUBTRACT
	};

	typedef struct SState {

		double x_;
		double y_;
		double z_;
		double roll_;
		double pitch_;
		double yaw_;
	};

	vector<SState> state_vec;

	//input txt
	{
		vector<vector<double>> trajectory_vec_vec;
		trajectory_vec_vec = time_.getVecVecFromCSV<double>(filename_);
		for (int i = 0; i < trajectory_vec_vec.size(); i++)
		{
			SState state;
			state.x_ = trajectory_vec_vec[i][1];
			state.y_ = trajectory_vec_vec[i][2];
			state.z_ = trajectory_vec_vec[i][3];
			state.roll_ = trajectory_vec_vec[i][4];
			state.pitch_ = trajectory_vec_vec[i][5];
			state.yaw_ = trajectory_vec_vec[i][6];
			state_vec.push_back(state);
		}
	}
	cout << "state_vec size = " << state_vec.size() << endl;

	KEYNUM key_;
	double x_delta, y_delta, z_delta, roll_delta, pitch_delta, yaw_delta;
	double x_delta_init, y_delta_init, z_delta_init, roll_delta_init, pitch_delta_init, yaw_delta_init;
	x_delta_init = y_delta_init = z_delta_init = roll_delta_init = pitch_delta_init = yaw_delta_init = 0.;

	while (1) {


		//input next PointCloud
		if (b_makeNewPC) {

			cloud_moving_before->clear();

			string filename_PC;
			filename_PC = to_string(num_PC_now);
			if (filename_PC.size() < 3) filename_PC = "0" + filename_PC;
			if (filename_PC.size() < 3) filename_PC = "0" + filename_PC;
			filename_PC = foldername_ + filename_PC + ".pcd";

			if (-1 == pcl::io::loadPCDFile(filename_PC, *cloud_moving_before)) break;

			cout << "PC(" << num_PC_now << ") number :" << cloud_moving_before->size() << endl;

			cout << endl;
			cout << "**********( key option )**********" << endl;
			cout << "Use numpad" << endl;
			cout << " TURN_LEFT:7    UP:8  TURN_RIGHT:9" << endl;
			cout << "      LEFT:4    ----       RIGHT:6" << endl;
			cout << "      ------  DOWN:2       -------" << endl;

			cout << endl;
			cout << "Reset:-(numpad)" << endl;

			cout << "Switch:ENTER" << endl;
			cout << "Escape:ESC" << endl;
			cout << "**********************************" << endl;
			cout << endl;

			//state_vec
			x_delta_init = state_vec[num_PC_now].x_;
			y_delta_init = state_vec[num_PC_now].y_;
			z_delta_init = state_vec[num_PC_now].z_;
			roll_delta_init = state_vec[num_PC_now].roll_;
			pitch_delta_init = state_vec[num_PC_now].pitch_;
			yaw_delta_init = state_vec[num_PC_now].yaw_;

			x_delta = x_delta_init;
			y_delta = y_delta_init;
			z_delta = z_delta_init;
			roll_delta = roll_delta_init;
			pitch_delta = pitch_delta_init;
			yaw_delta = yaw_delta_init;

			cloud_moving->clear();
			Trans_ = Eigen::Affine3f::Identity();
			Trans_.translation() << x_delta, y_delta, z_delta;
			Trans_.rotate(Eigen::AngleAxisf(roll_delta, Eigen::Vector3f::UnitX()));
			Trans_.rotate(Eigen::AngleAxisf(pitch_delta, Eigen::Vector3f::UnitY()));
			Trans_.rotate(Eigen::AngleAxisf(yaw_delta, Eigen::Vector3f::UnitZ()));
			pcl::transformPointCloud(*cloud_moving_before, *cloud_moving, Trans_);

			//ground
			bool b_RemoveGround;
			b_RemoveGround = true;
			if(b_RemoveGround)
			{
				double th_height;
				th_height = -0.1;
				cloud_temp->clear();
				pcl::copyPointCloud(*cloud_moving, *cloud_temp);
				cloud_moving->clear();
				for (size_t i = 0; i < cloud_temp->size(); i++)
				{
					if (th_height > cloud_temp->points[i].z) continue;
					cloud_moving->push_back(cloud_temp->points[i]);
				}
			}

			b_makeNewPC = false;

		}

		//https://www.slideshare.net/masafuminoda/pcl-11030703
		//Viewer
		//左ドラッグ：視点の回転
		//Shift+左ドラッグ：視点の平行移動．
		//Ctrl+左ドラッグ：画面上の回転
		//右ドラッグ：ズーム
		//g：メジャーの表示
		//j：スクリーンショットの保存

		//input key
		short key_num_up = GetAsyncKeyState(VK_NUMPAD8);
		short key_num_down = GetAsyncKeyState(VK_NUMPAD2);
		short key_num_right = GetAsyncKeyState(VK_NUMPAD6);
		short key_num_left = GetAsyncKeyState(VK_NUMPAD4);
		short key_num_turn_r = GetAsyncKeyState(VK_NUMPAD9);
		short key_num_turn_l = GetAsyncKeyState(VK_NUMPAD7);
		short key_num_enter = GetAsyncKeyState(VK_RETURN);
		short key_num_escape = GetAsyncKeyState(VK_ESCAPE);
		short key_num_subt_numpad = GetAsyncKeyState(VK_SUBTRACT);

		if ((key_num_up & 1) == 1) key_ = UP;
		else if ((key_num_down & 1) == 1) key_ = DOWN;
		else if ((key_num_right & 1) == 1) key_ = RIGHT;
		else if ((key_num_left & 1) == 1) key_ = LEFT;
		else if ((key_num_turn_r & 1) == 1) key_ = TURN_R;
		else if ((key_num_turn_l & 1) == 1) key_ = TURN_L;
		else if ((key_num_enter & 1) == 1) key_ = ENTER;
		else if ((key_num_subt_numpad & 1) == 1) key_ = SUBTRACT;
		else if ((key_num_escape & 1) == 1)
		{
			cout << "ESC called" << endl;
			b_escaped = true;
			break;
		}
		else key_ = NONE;

		if (b_first)
		{
			key_ = NONE;
			b_first = false;
		}

		if(key_ != NONE) cout << "key_ = " << key_ << endl;

		//determine transformation by key
		switch (key_) {

		case UP:
			y_delta += disp_translation;
			break;

		case DOWN:
			y_delta -= disp_translation;
			break;

		case RIGHT:
			x_delta += disp_translation;
			break;

		case LEFT:
			x_delta -= disp_translation;
			break;

		case TURN_R:
			yaw_delta -= disp_rotation;
			break;

		case TURN_L:
			yaw_delta += disp_rotation;
			break;

		case ENTER:
		{

			*cloud_show_static += *cloud_moving;

			state_vec[num_PC_now].x_ = x_delta;
			state_vec[num_PC_now].y_ = y_delta;
			state_vec[num_PC_now].z_ = z_delta;
			state_vec[num_PC_now].roll_ = roll_delta;
			state_vec[num_PC_now].pitch_ = pitch_delta;
			state_vec[num_PC_now].yaw_ = yaw_delta;

			num_PC_now++;
			b_makeNewPC = true;
			cout << "ENTER pressed" << endl;

			break;
		}

		case SUBTRACT:
		{
			x_delta = x_delta_init;
			y_delta = y_delta_init;
			z_delta = z_delta_init;
			roll_delta = roll_delta_init;
			pitch_delta = pitch_delta_init;
			yaw_delta = yaw_delta_init;
			cout << "-(numpad) pressed" << endl;

		}

		default:
			break;

		}

		if (!(key_ == NONE || key_ == ENTER)) {

			cloud_moving->clear();

			Trans_ = Eigen::Affine3f::Identity();
			Trans_.translation() << x_delta, y_delta, z_delta;
			Trans_.rotate(Eigen::AngleAxisf(roll_delta, Eigen::Vector3f::UnitX()));
			Trans_.rotate(Eigen::AngleAxisf(pitch_delta, Eigen::Vector3f::UnitY()));
			Trans_.rotate(Eigen::AngleAxisf(yaw_delta, Eigen::Vector3f::UnitZ()));
			pcl::transformPointCloud(*cloud_moving_before, *cloud_moving, Trans_);

		}
		cloud_show->clear();
		*cloud_show += *cloud_show_static;
		*cloud_show += *cloud_moving;

		if (cloud_show->size() != 0) {
			//cout << "show!" << endl;
			ShowPcdFile(cloud_show);
		}
	}

	//output txt
	if (!b_escaped)
	{
		vector<vector<double>> trajectory_vec_vec;
		for (int i = 0; i < state_vec.size(); i++)
		{
			vector<double> trajectory_vec;
			trajectory_vec.push_back(i);
			trajectory_vec.push_back(state_vec[i].x_);
			trajectory_vec.push_back(state_vec[i].y_);
			trajectory_vec.push_back(state_vec[i].z_);
			trajectory_vec.push_back(state_vec[i].roll_);
			trajectory_vec.push_back(state_vec[i].pitch_);
			trajectory_vec.push_back(state_vec[i].yaw_);
			trajectory_vec_vec.push_back(trajectory_vec);
		}
		time_.getCSVFromVecVec(trajectory_vec_vec,filename_);
		cout << "file has saved!" << endl;
	}
	else cout << "file has not saved!" << endl;
}


void CVelodyneInterface::ShowOnlySequent(string foldername_) {

	enum OPTION_SEQUENT {
		EN_ReadBefore = 1,
		EN_DifferentName_summer,
		EN_DifferentName_winter
	};

	OPTION_SEQUENT option_;
	//option_ = EN_ReadBefore;
	option_ = EN_DifferentName_winter;

	if (option_ == EN_ReadBefore) SequentBeforeRead_2019Naraha(foldername_);

	else if (option_ == EN_DifferentName_summer) SequentDifferentName_2019Naraha(foldername_);

	else if (option_ == EN_DifferentName_winter) SequentDifferentName_2020Naraha(foldername_);

	cout << "finished" << endl;


	//don't know finish filename
	//if through filename which is not, infinit loop occur,so I should preread final filename

}


void CVelodyneInterface::getOnlyShow3(vector<CRobotState> state_arg) {
	//for (int i = 0; i < 181; i++) {
	//	cout << "RangeData[" << i - 90 << "] = " << vector_arg[i] << endl;

	//}
	for (int i = 0; i < state_arg.size(); i++) {
		cout << "StateDataX[" << (int)(i - (state_arg.size() - 1) / 2) << "] = " << state_arg[i].x << endl;
		cout << "StateDataY[" << (int)(i - (state_arg.size() - 1) / 2) << "] = " << state_arg[i].y << endl << endl;;

	}

	getchar();
}


void CVelodyneInterface::getOnlyShow2(vector<double> vector_arg) {
	//for (int i = 0; i < 181; i++) {
	//	cout << "RangeData[" << i - 90 << "] = " << vector_arg[i] << endl;

	//}
	for (int i = 0; i < vector_arg.size(); i++) {
		cout << "RangeData[" << (int)(i - (vector_arg.size()-1)/2) << "] = " << vector_arg[i] << endl;

	}


	//double *Array;
	//Array = new double[181];
	//for (int i = 0; i < 181; i++) {

	//	Array[i] = 1.;
	//}

	//Array = ;

	//for (int i = 0; i < 181; i++) {

	//	cout << "Range[" << i << "] = " << Array[i] << endl;
	//}

	getchar();
}


void CVelodyneInterface::getOnlyShow(double* array_arg) {
	for (int i = 0; i < 181; i++) {
		cout << "RangeData[" << i-90 << "] = " << array_arg[i] << endl;

	}
	//double *Array;
	//Array = new double[181];
	//for (int i = 0; i < 181; i++) {

	//	Array[i] = 1.;
	//}

	//Array = ;

	//for (int i = 0; i < 181; i++) {

	//	cout << "Range[" << i << "] = " << Array[i] << endl;
	//}

	getchar();
}


vector<double> CVelodyneInterface::getDegVector(pcl::PointCloud<PointType>::Ptr cloud,int NumOfData) {
	//vector<CRobotState::Position> PositionVector;
	vector<double> RangeVector;
	vector<double> MinErrorArray;
	//auto Itr = RangeVector_181.begin();

	//debug
	CTimeString ts;
	cout << "start at " << ts.getTimeString() << endl;

	double resolution = 180. / (NumOfData - 1);//resolution		181->1deg, 361->0.5deg 



	//initialize
	for (int i = 0; i < NumOfData; i++) {
		//CRobotState::Position position_;
		//position_.x = 100.;
		//position_.y = 0.;
		//position_x = 0.;
		//PositionVector.push_back(position_)

		RangeVector.push_back(100.);		//Max Range
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
		if(angle_deg>=0) standard_num = (int)((angle_deg + resolution / 2.) / resolution);
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

			RangeVector[standard_num + (NumOfData - 1) / 2] = sqrt(pow(cloud->points[i].x, 2.) + pow(cloud->points[i].y, 2.));

		}
	}

	cout << "end at " << ts.getTimeString() << endl;


	return RangeVector;
}


double* CVelodyneInterface::convertPCto181DegArray(pcl::PointCloud<PointType>::Ptr cloud) {

	double *RangeArray_181;
	RangeArray_181 = new double[181];		//Can this result memory leak ?
	//double RangeArray_181[181];

	double MinErrorArray_181[181];

	for (int i = 0; i < 181; i++) {
		RangeArray_181[i] = 100.;		
		MinErrorArray_181[i] = 1.;
	}

	for (int i = 0; i < cloud->size(); i++)	{

		double angle_deg;
		int standard;		// 1.0, 2.0, 3.0, ...(integer)
		double error;		//distance from angle to standard

		angle_deg = atan2(cloud->points[i].x, cloud->points[i].y)*180. / M_PI;

		// convertion from clockwise to anticlockwise
		angle_deg = -angle_deg + 90.;
		if (angle_deg >= 180.) angle_deg -= 360.;

		//debug
		//cout << "range["<<angle_deg<<"] =" << sqrt(pow(cloud->points[i].x, 2.) + pow(cloud->points[i].y, 2.)) << endl;

		if (!(-91. < angle_deg && angle_deg < 91.)) continue;

		standard = (int)(angle_deg + 0.5);

		error = fabs(angle_deg - standard);

		if (MinErrorArray_181[standard + 90] > error) {
			MinErrorArray_181[standard + 90] = error;
			RangeArray_181[standard + 90] = sqrt(pow(cloud->points[i].x,2.)+ pow(cloud->points[i].y, 2.));
			
		}

	}

	return RangeArray_181;
}


void CVelodyneInterface::show_2D() {

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

		short key_num_espace = GetAsyncKeyState(VK_ESCAPE);
		if ((key_num_espace & 1) == 1) {
			cout << "toggled!" << endl;
			break;
		}

	}
}

pcl::PointCloud<PointType>::Ptr CVelodyneInterface::getPointCloud() {
	boost::mutex::scoped_lock lock(m_mutex);
	pcl::PointCloud<PointType>::Ptr PointCloud(new pcl::PointCloud<PointType>(*m_PointCloud_ConstPtr));	//ConstPtr -> Ptr
	return PointCloud;

}

pcl::PointCloud<PointType>::Ptr CVelodyneInterface::get2DPointCloud() {
	boost::mutex::scoped_lock lock(m_mutex);
	pcl::PointCloud<PointType>::Ptr cloud_ptr(new pcl::PointCloud<PointType>(*m_PointCloud_ConstPtr));	//ConstPtr -> Ptr
	return ElevationAngleFiltering(cloud_ptr);
}

pcl::PointCloud<PointType>::Ptr CVelodyneInterface::get2DPointCloud_(boost::mutex &mutex_arg) {
	boost::mutex::scoped_lock lock(mutex_arg);
	pcl::PointCloud<PointType>::Ptr cloud_ptr(new pcl::PointCloud<PointType>(*m_PointCloud_ConstPtr));	//ConstPtr -> Ptr
	return ElevationAngleFiltering(cloud_ptr);
}


pcl::PointCloud<PointType>::ConstPtr CVelodyneInterface::getPointCloud_ConstPtr() {
	boost::mutex::scoped_lock lock(m_mutex);
	return m_PointCloud_ConstPtr;
}

void CVelodyneInterface::ReadAndShow2Clouds(string filename1, string filename2) {

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;				//大丈夫？
	viewer.reset(new pcl::visualization::PCLVisualizer("Velodyne Viewer"));
	viewer->initCameraParameters();

	pcl::visualization::PointCloudColorHandler<PointType>::Ptr handler;

	int v1 = 0;
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->addCoordinateSystem(3.0, "coordinate", v1);
	viewer->setBackgroundColor(0.0, 0.0, 0.0, v1);
	viewer->setCameraPosition(0.0, 0.0, 30.0, 0.0, 1.0, 0.0, v1);

	int v2 = 0;
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->addCoordinateSystem(3.0, "coordinate", v2);
	viewer->setBackgroundColor(0.05, 0.05, 0.05, v2);
	viewer->setCameraPosition(0.0, 0.0, 30.0, 0.0, 1.0, 0.0, v2);

	const std::type_info& type = typeid(PointType);
	if (type == typeid(pcl::PointXYZ)) {
		std::vector<double> color = { 255.0, 255.0, 255.0 };
		boost::shared_ptr<pcl::visualization::PointCloudColorHandlerCustom<PointType>> color_handler(new pcl::visualization::PointCloudColorHandlerCustom<PointType>(color[0], color[1], color[2]));
		handler = color_handler;
	}
	else if (type == typeid(pcl::PointXYZI)) {
		boost::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<PointType>> color_handler(new pcl::visualization::PointCloudColorHandlerGenericField<PointType>("intensity"));
		handler = color_handler;
	}
	else if (type == typeid(pcl::PointXYZRGBA)) {
		boost::shared_ptr<pcl::visualization::PointCloudColorHandlerRGBField<PointType>> color_handler(new pcl::visualization::PointCloudColorHandlerRGBField<PointType>());
		handler = color_handler;
	}
	else {
		throw std::runtime_error("This PointType is unsupported.");
	}

	cout << "wait for reading..." << endl;
	pcl::PointCloud<PointType> cloud1;
	pcl::io::loadPCDFile(filename1, cloud1);
	pcl::PointCloud<PointType>::Ptr cloud1c(new pcl::PointCloud<PointType>());
	cloud1c = cloud1.makeShared();
	handler->setInputCloud(cloud1c);
	if (!viewer->updatePointCloud(cloud1c, *handler, "cloud1")) {
		viewer->addPointCloud(cloud1c, *handler, "cloud11", v1);
	}

	pcl::PointCloud<PointType> cloud2;
	pcl::io::loadPCDFile(filename2, cloud2);
	pcl::PointCloud<PointType>::Ptr cloud2c(new pcl::PointCloud<PointType>());
	cloud2c = cloud2.makeShared();
	handler->setInputCloud(cloud2c);
	if (!viewer->updatePointCloud(cloud2c, *handler, "cloud2")) {
		viewer->addPointCloud(cloud2c, *handler, "cloud2", v2);
	}
	cout << "succeeded reading files!" << endl;

	while (1) {
		viewer->spinOnce();
		short key_num_espace = GetAsyncKeyState(VK_ESCAPE);
		if ((key_num_espace & 1) == 1) {
			cout << "toggled!" << endl;
			break;
		}

	}
}


pcl::PointCloud<PointType>::Ptr CVelodyneInterface::ElevationAngleFiltering(pcl::PointCloud<PointType>::Ptr cloud) {

	pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>());

	//CTimeString time;

	//cout << "first points.size = " << cloud->points.size() << endl;
	//cout << "and now" << time.getTimeStirng() << endl;
	for (size_t i = 0; i < cloud->points.size(); i++) {
			double sin_elevation = cloud->points[i].z / sqrt(pow(cloud->points[i].x, 2.) + pow(cloud->points[i].y, 2.) + pow(cloud->points[i].z, 2.));
			double angle_ = asin(sin_elevation);

			//if (angle_AbsoluteValue < 1.5*(M_PI/180.)) {
			if(0 < angle_ && angle_ < 1.5*(M_PI/180)){
				//cast in horizontal plane
				cloud->points[i].z = 0.;
				cloud_filtered->points.push_back(cloud->points[i]);
				//cout << "added" << endl;

			}
			//else cout << "not added" << endl;
				
	}
	//cout << "result point.size = " << cloud_filtered->points.size() << endl;
	//cout << "and now" << time.getTimeStirng() << endl;

	//process to prevent error occuring
	cloud_filtered->width = 1;
	cloud_filtered->height = cloud_filtered->points.size();

	return cloud_filtered;
}

void CVelodyneInterface::ReadAndShowOne(string filename_arg) {

	pcl::PointCloud<PointType> cloud;
	pcl::io::loadPCDFile(filename_arg, cloud);									//PtrではなくPointCloudで保存．Ptrの参照外しは渡せない．
	cout << "readed" << endl;

	////plate mode
	//pcl::PointCloud<PointType>::Ptr cloud_filtered;
	//cloud_filtered = ElevationAngleFiltering(cloud.makeShared());				//PointCloud -> Ptr
	//////save
	////if (cloud_filtered) {
	////	cout << "found" << endl;
	////	pcl::io::savePCDFile<PointType>("\../savedfolder/20181017_0242_45_080_cut.pcd", *cloud_filtered);		//ポインタではなく値で保存
	////}
	////else {
	////	cout << "not found" << endl;
	////	return;
	////}
	////showing
	//while (!m_viewer->wasStopped()) {
	//	ShowPcdFile(cloud_filtered);
	//	if (toggle.isToggleChanged()) {
	//		cout << "Toggled!" << endl;
	//		break;
	//	}
	//}


	////
	////getOnlyShow(convertPCto181DegArray(cloud_filtered));
	//double* array_;
	//array_ = new double[181];
	//array_ = convertPCto181DegArray(cloud_filtered);
	//getOnlyShow(array_);
	//delete array_;

	//getOnlyShow2(getDegVector(cloud_filtered,361));

	//getOnlyShow3(getDegStateVector(cloud_filtered,361));


	//normal mode
	while (!m_viewer->wasStopped()) {
		ShowPcdFile(cloud.makeShared());
		short key_num_espace = GetAsyncKeyState(VK_ESCAPE);
		if ((key_num_espace & 1) == 1) {
			cout << "toggled!" << endl;
			break;
		}

	}

	////spin mode
	//m_handler->setInputCloud(cloud.makeShared());
	//m_viewer->updatePointCloud(cloud.makeShared(), *m_handler, "cloud");
	//m_viewer->addPointCloud(cloud.makeShared(), *m_handler, "cloud");
	//m_viewer->spin();

	cout << "viewer closed" << endl;
}

void CVelodyneInterface::ShowPcdFile(pcl::PointCloud<PointType>::Ptr p_cloud) {

	m_viewer->spinOnce();

	//ファイルのポインタを受け取るようにする？
	if (p_cloud) {
		//pcl::PointCloud<PointType>::ConstPtr cloud = p_cloud;					//なぜ代入できた？

		// Update Point Cloud									//値は渡せない？
		m_handler->setInputCloud(p_cloud);
		if (!m_viewer->updatePointCloud(p_cloud, *m_handler, "cloud")) {
			m_viewer->addPointCloud(p_cloud, *m_handler, "cloud");
			cout << "succeeded showing" << endl;
		}

	}
	else cout << "fail to show" << endl;
}

void CVelodyneInterface::TimerWrite() {

	CTimeString time;
	int wait_second = 10;
	cout << "Please write waiting time[s]:";
	cin >> wait_second;

	while (1) {
		cout << "waiting..." << endl;
		Sleep(10);
		short key_num = GetAsyncKeyState(VK_SPACE);
		if ((key_num & 1) == 1) {
			break;
		}

	}
	cout << "Toggled and start!!" << endl;
	Sleep(wait_second * 1000);

	while (1) {
		boost::mutex::scoped_try_lock lock(m_mutex);
		if (lock.owns_lock() && m_PointCloud_ConstPtr) {
			pcl::io::savePCDFile<PointType>("\../savedfolder/" + time.getTimeString() + ".pcd", *m_PointCloud_ConstPtr);
			cout << "1 Written!" << endl;
			break;
		}
	}

	//Sleep(10*1000);
	//
	//while (1) {
	//	boost::mutex::scoped_try_lock lock(m_mutex);
	//	if (lock.owns_lock() && m_PointCloud_ConstPtr) {
	//		pcl::io::savePCDFile<PointType>("\../savedfolder/" + time.getTimeString() + ".pcd", *m_PointCloud_ConstPtr);
	//		cout << "2 Written!" << endl;
	//		break;
	//	}
	//}

}


void CVelodyneInterface::ToggleWrite() {

	bool b_AttemptCapture = false;

	CTimeString time;

	int num_data = 0;

	////描画のループ
	cout << "Press SPACE to screenshot" << endl;
	while (!m_viewer->wasStopped()) {
		// Update Viewer
		m_viewer->spinOnce();

		{
			short key_num = GetAsyncKeyState(VK_SPACE);
			if ((key_num & 1) == 1) {
				b_AttemptCapture = true;
				cout << "toggled!" << endl;
			}
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
				filename_ = time.getTimeString() + ".pcd";
				//filename_ = to_string(num_data);
				//if (filename_.size() < 4) filename_ = "0" + filename_;
				//if (filename_.size() < 4) filename_ = "0" + filename_;
				//if (filename_.size() < 4) filename_ = "0" + filename_;
				//filename_ += ".pcd";

				cout << filename_ << endl;
				pcl::io::savePCDFile<PointType>("\../savedfolder/" + filename_, *m_PointCloud_ConstPtr);
				b_AttemptCapture = false;
				cout << "↑Written!" << endl;
				num_data++;

			}
		}

		short key_num_espace = GetAsyncKeyState(VK_ESCAPE);
		if ((key_num_espace & 1) == 1) {

			cout << "Finished!" << endl;
		}


	}

}

void CVelodyneInterface::Process() {

	initVisualizer();
	
	//connect();

	//描画をする関数
	show();

	disconnect();

}

void CVelodyneInterface::show() {

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

		short key_num = GetAsyncKeyState(VK_SPACE);
		if ((key_num & 1) == 1) {

			cout << "toggled!" << endl;
		}

	}

}

void CVelodyneInterface::initVisualizer() {

	// PCL Visualizer
	m_viewer.reset(new pcl::visualization::PCLVisualizer("Velodyne Viewer"));

	m_viewer->addCoordinateSystem(3.0, "coordinate");
	m_viewer->setBackgroundColor(0.0, 0.0, 0.0, 0);
	m_viewer->initCameraParameters();
	m_viewer->setCameraPosition(0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);

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
	else {
		throw std::runtime_error("This PointType is unsupported.");
	}


}

//センサデータ取得のスレッドを走らせるための関数
void CVelodyneInterface::execute(int nPeriod) {

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

void CVelodyneInterface::SequentBeforeRead_2019Naraha(string foldername_)
{
	vector<pcl::PointCloud<PointType>::Ptr> cloud_read_vec;
	//read all files
	int num_read = 0;
	while (1) {
		cout << "reading :" << num_read << endl;
		string filename_;
		filename_ = to_string(num_read);
		if (filename_.size() < 3) filename_ = "0" + filename_;
		if (filename_.size() < 3) filename_ = "0" + filename_;
		filename_ = foldername_ + filename_ + ".pcd";
		pcl::PointCloud<PointType>::Ptr cloud_(new pcl::PointCloud<PointType>());
		if (-1 == pcl::io::loadPCDFile(filename_, *cloud_)) break;
		cloud_read_vec.push_back(cloud_);

		cout << "PC(" << num_read << ") number :" << cloud_->size() << endl;

		num_read++;
	}
	//show all PointCloud
	cout << "show all PointCloud" << endl;
	cout << "Press SPACE to switch" << endl;
	int index = 0;
	cout << "showing :" << 0 << endl;
	while (!m_viewer->wasStopped()) {
		//change PointCloud
		short key_num = GetAsyncKeyState(VK_SPACE);
		if ((key_num & 1) == 1) {
			if (index + 1 < num_read)	index++;
			cout << "showing :" << index << endl;
		}
		ShowPcdFile(cloud_read_vec[index]);
		//escape
		short key_num_esc = GetAsyncKeyState(VK_ESCAPE);
		if ((key_num_esc & 1) == 1) {
			cout << "toggled!" << endl;
			break;
		}
	}

}

void CVelodyneInterface::SequentDifferentName_2019Naraha(string foldername_) 
{
	int num_PC = 14;
	int num_read = 0;
	int index = 0;
	bool b_first = true;
	pcl::PointCloud<PointType>::Ptr cloud_(new pcl::PointCloud<PointType>());
	while (!m_viewer->wasStopped()) {
		//change PointCloud
		short key_num = GetAsyncKeyState(VK_SPACE);
		if (((key_num & 1) == 1) || b_first) {
			//if(!b_first && index + 1 < num_PC)	index++;
			if (!b_first) {
				if (index + 1 < num_PC) index++;
				else {
					cout << "Error! : PointCloud number" << endl;
					continue;
				}
			}
			cout << "reading :" << index << endl;
			string filename_;
			filename_ = to_string(index);
			if (filename_.size() < 3) filename_ = "0" + filename_;
			if (filename_.size() < 3) filename_ = "0" + filename_;
			filename_ = foldername_ + filename_ + ".pcd";
			pcl::io::loadPCDFile(filename_, *cloud_);
			cout << "showing :" << index << endl;
			b_first = false;
		}
		ShowPcdFile(cloud_);
		//escape
		short key_num_esc = GetAsyncKeyState(VK_ESCAPE);
		if ((key_num_esc & 1) == 1) {
			cout << "toggled!" << endl;
			break;
		}
	}
}

void CVelodyneInterface::SequentDifferentName_2020Naraha(string foldername_)
{
	foldername_ = "\../savedfolder/20200119/PointCloud/";

	bool b_read_text = false;
	b_read_text = true;

	//20200119_1732_46_243
	//20200119_1753_30_556

	//初回以外はindexに追加して読み込んでいた．
	//こちらでは統一したい．

	//とりあえずスタートから総当たりで読み込んでいく．総当たり時の文字列生成．
	//外れだったら次の文字列を生成．外れ判定はif (-1 == pcl::io::loadPCDFile(filename_, *cloud_)) break;

	//スペースが押されたら読み込む．
	//このタイミングで名前のサーチをする．

	pcl::PointCloud<PointType>::Ptr cloud_(new pcl::PointCloud<PointType>());

	string filename_start = "20200119_1732_46_243.pcd";
	string filename_end = "20200119_1753_30_556.pcd";


	if (b_read_text)
	{
		CTimeString time_;
		vector<string> filename_vec;
		bool b_first = true;
		int index_ = 0;

		ifstream ifs_(foldername_+ "_ExistingFiles.txt");
		string str_;
		if (ifs_.fail()) cout << "Error: file could not be read." << endl;
		else {
			while (getline(ifs_, str_))
			{
				if(str_.size()>2) filename_vec.push_back(str_);
			}
			ifs_.close();
		}

		cout << "file number:" << filename_vec.size() << endl;

		////最後のは余分．読み込み時の列数で対応

		while (!m_viewer->wasStopped()) {
			//change PointCloud
			short key_num = GetAsyncKeyState(VK_SPACE);
			if ((key_num & 1) == 1) {
				pcl::io::loadPCDFile(foldername_ + filename_vec[index_], *cloud_);
				cout << "showing:" << filename_vec[index_] << endl;

				index_++;
				if (index_ > filename_vec.size())
				{
					cout << "index over" << endl;
					break;
				}

			}
			ShowPcdFile(cloud_);
			//escape
			short key_num_esc = GetAsyncKeyState(VK_ESCAPE);
			if ((key_num_esc & 1) == 1) {
				cout << "toggled!" << endl;
				break;
			}
		}
	}

	else
	{

		bool b_first = true;
		int index = 0;

		////20200119_1753_29_886.pcd
		//int i_minute = 53;
		//int i_second = 29;
		//int i_millisecond = 885;


		int i_minute = 34;
		int i_second = 10;
		int i_millisecond = 0;

		//int i_minute = 32;
		//int i_second = 0;
		//int i_millisecond = 0;
		string filename_;
		string filename_allfound;

		while (!m_viewer->wasStopped())
		{

			if (b_first)
			{
				pcl::io::loadPCDFile(foldername_ + filename_start, *cloud_);
				b_first = false;
				index++;
				filename_allfound += filename_start + "\n";

			}
			else
			{
				short key_num = GetAsyncKeyState(VK_SPACE);

				if (1)
					//if ((key_num & 1) == 1)
				{
					bool b_found = false;

					//name search
					//minute:32-60, second:00-60, millisecond:000-999
					//20200119_1734_10_989.pcd
					do
					{
						string s_minute;
						s_minute = to_string(i_minute);
						if (s_minute.size() < 2) s_minute = "0" + s_minute;

						do
						{
							string s_second;
							s_second = to_string(i_second);
							if (s_second.size() < 2) s_second = "0" + s_second;

							do
							{
								string s_millisecond;
								s_millisecond = to_string(i_millisecond);
								if (s_millisecond.size() < 3) s_millisecond = "0" + s_millisecond;
								if (s_millisecond.size() < 3) s_millisecond = "0" + s_millisecond;

								filename_ = s_minute + "_" + s_second + "_" + s_millisecond + ".pcd";

								//filename_
								if (-1 != pcl::io::loadPCDFile(foldername_ + "20200119_17" + filename_, *cloud_))	//found
								//if (-1 != pcl::io::loadPCDFile(filename_temp, *cloud_))	//found
								{
									b_found = true;
									cout << "Find:" << "20200119_17" + filename_ << endl;
									index++;
									filename_allfound += "20200119_17" + filename_ + "\n";
									cout << "show all found file:" << endl;
									cout << filename_allfound << endl;
								}
								cout << "index:" << index << endl;

								cout << endl;

								i_millisecond++;
								if (b_found) break;
							} while (i_millisecond < 1000);
							if (i_millisecond == 1000)
							{
								i_millisecond = 0;
								i_second++;
							}

							if (b_found) break;
						} while (i_second < 60);
						if (i_second == 60)
						{
							i_second = 0;
							i_minute++;
						}

						if (b_found) break;
					} while (i_minute < 60);
					if (i_minute == 60) i_minute = 0;

				}


			}

			if (filename_end.compare("20200119_17" + filename_) == 0)
			{
				cout << "final file" << endl;
				std::ofstream ofs_save;
				ofs_save.open(foldername_ + "_ExistingFiles.txt", std::ios::out);
				ofs_save << filename_allfound;
				ofs_save.close();
				break;
			}


			ShowPcdFile(cloud_);
			//escape
			short key_num_esc = GetAsyncKeyState(VK_ESCAPE);
			if ((key_num_esc & 1) == 1) {
				cout << "toggled!" << endl;
				break;
			}
		}

	}



}

