#include"Velodyne_Interface.h"

void CVelodyneInterface::all(string ipaddress, string port)
{
	int WhichProcess = 0;
	string filename1, filename2;
	bool b_finish = false;

	enum OPTION {
		EN_escape = 0,
		EN_FreeSpace,
		EN_ReadAndShowOne,
		EN_show2PointClouds,
		EN_ToggleWrite,
		EN_TimerWrite,
		EN_capture,
		EN_capture2D,
		EN_sequentshow,
		EN_handregistration,
		EN_GetPcdFromCSV,
	};

	while (!b_finish)
	{
		cout << endl;
		cout << "please input process number" << endl;
		cout << EN_escape << ": escape" << endl;
		cout << EN_FreeSpace << ": free space" << endl;
		cout << EN_ReadAndShowOne << ": ReadAndShowOne" << endl;
		cout << EN_show2PointClouds << ": show 2 PointClouds" << endl;
		cout << EN_ToggleWrite << ": ToggleWrite" << endl;
		cout << EN_TimerWrite << ": TimerWrite" << endl;
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
			//initVisualizer();		//XYZRGB
			FreeSpace();
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
			//ShowOnlySequent("../savedfolder/naraha summer/sequent");
			//ShowOnlySequent("../savedfolder/20200119/PointCloud/");
			ShowOnlySequent("../savedfolder/temp");
			break;
		}

	}
	
	cout << "process finished (press:ESC)" << endl;
	GetAsyncKeyState(VK_ESCAPE);
	while (1)
	{
		if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) break;
	}

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
	//cout << "HandRegistration started!" << endl;

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_add(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sum(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZI>());

	pcl::ApproximateVoxelGrid<pcl::PointXYZI> VGFilter;
	pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> VGFilter_RGB;

	string foldername_ = "../savedfolder/temp";

	CTimeString time_;

	Eigen::Affine3f Trans_;
	Eigen::Matrix4d HM_free = Eigen::Matrix4d::Identity();

	int index_PC_now = 0;
	bool b_makeNewPC = true;
	bool b_escaped = false;
	bool b_break = false;

	vector<string> filenames_;
	time_.getFileNames_extension(foldername_, filenames_, ".pcd");

	std::string filename_txt;

	//input txt
	vector<vector<double>> trajectory_vec_vec;
	{
		vector<string> filenames__txt;
		time_.getFileNames_extension(foldername_, filenames__txt, ".txt");

		if (filenames__txt.size() == 0)
		{
			cout << "found no txt file." << endl;
			return;
		}
		else if (filenames__txt.size() == 1)
		{
			CTimeString::getCSVFromVecVec(trajectory_vec_vec, foldername_ + "/" + filenames__txt[0]);
			filename_txt = foldername_ + "/" + filenames__txt[0];
		}
		cout << "trajectory_vec_vec size = " << trajectory_vec_vec.size() << endl;
	}

	GetAsyncKeyState(VK_RETURN);

	//PointCloud
	while (1) {
		bool b_nir;
		if (index_PC_now >= 17) b_nir = true;
		else b_nir = false;
		//if (index_PC_now >= 1) b_nir = true;
		//else b_nir = false;


		//input next PointCloud
		if (b_makeNewPC) {
			cloud_->clear();

			string filename_PC;
			filename_PC = filenames_[index_PC_now];
			cout << endl;
			cout << "reanding: " << filename_PC << endl;
			filename_PC = foldername_ + "/" + filename_PC;
			if (-1 == pcl::io::loadPCDFile(filename_PC, *cloud_)) break;

			//turn pitch(camera axis)
			{
				double pitch_init;
				pitch_init = 22. * M_PI / 180.;
				HM_free = Eigen::Matrix4d::Identity();
				HM_free = calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., pitch_init, 0.);
				Trans_ = Eigen::Affine3f::Identity();
				Trans_ = calcAffine3fFromHomogeneousMatrix(HM_free);
				pcl::transformPointCloud(*cloud_, *cloud_, Trans_);
			}

			//ground
			bool b_RemoveGround = false;
			if(!b_nir) b_RemoveGround = true;		//nir
			if (b_RemoveGround)
			{
				double th_height;
				//th_height = -0.1;	//naraha summer
				th_height = -0.3;
				cloud_temp->clear();
				pcl::copyPointCloud(*cloud_, *cloud_temp);
				cloud_->clear();
				for (size_t i = 0; i < cloud_temp->size(); i++)
				{
					if (th_height > cloud_temp->points[i].z) continue;
					cloud_->push_back(cloud_temp->points[i]);
				}
			}

			//range
			if(b_nir)
			{
				bool b_modify = false;
				b_modify = true;
				double th_x, th_z;
				th_x = 3.;
				th_z = -1.2;
				//th_x = 10.;
				//th_z = -100.;
				cloud_temp->clear();
				pcl::copyPointCloud(*cloud_, *cloud_temp);
				cloud_->clear();
				for (int i = 0; i < cloud_temp->size(); i++)
				{
					pcl::PointXYZI point_ = cloud_temp->points[i];
					if (point_.x > th_x && b_modify) continue;
					if (point_.z < th_z && b_modify) continue;

					cloud_->push_back(point_);
				}
			}



			cout << "PC(" << index_PC_now << ") number :" << cloud_->size() << endl;

			cout << "VGF" << endl;
			double th_VGF = 0.01;
			VGFilter.setInputCloud(cloud_);
			VGFilter.setLeafSize(th_VGF, th_VGF, th_VGF);
			VGFilter.filter(*cloud_);

			cout << "PC(" << index_PC_now << ") number :" << cloud_->size() << endl;

			Trans_ = Eigen::Affine3f::Identity();
			HM_free = Eigen::Matrix4d::Identity();
			HM_free = calcHomogeneousMatrixFromVector6d(
				trajectory_vec_vec[index_PC_now][1],
				trajectory_vec_vec[index_PC_now][2],
				trajectory_vec_vec[index_PC_now][3],
				trajectory_vec_vec[index_PC_now][4],
				trajectory_vec_vec[index_PC_now][5],
				trajectory_vec_vec[index_PC_now][6]);
			Trans_ = calcAffine3fFromHomogeneousMatrix(HM_free);
			pcl::transformPointCloud(*cloud_, *cloud_, Trans_);

			b_makeNewPC = false;
		}

		//if ((GetAsyncKeyState(VK_RETURN) & 1) == 1)
		if (1)
		{
			cout << "ENTER pressed" << endl;

			b_makeNewPC = true;

			cloud_add->clear();
			if (b_nir)
			{
				for (int i = 0; i < cloud_->size(); i++)
				{
					pcl::PointXYZRGB point_;
					point_.x = cloud_->points[i].x;
					point_.y = cloud_->points[i].y;
					point_.z = cloud_->points[i].z;
					point_.r = 0;
					point_.g = (unsigned char)(cloud_->points[i].intensity);//0-255
					point_.b = 0;
					cloud_add->push_back(point_);
				}
			}
			else
			{
				//double max_ = 0.;
				//for (int i = 0; i < cloud_->size(); i++)
				//	if (max_ < cloud_->points[i].intensity) max_ = cloud_->points[i].intensity;
				//cout << "max_ = " << max_ << endl;
				for (int i = 0; i < cloud_->size(); i++)
				{
					pcl::PointXYZRGB point_;
					point_.x = cloud_->points[i].x;
					point_.y = cloud_->points[i].y;
					point_.z = cloud_->points[i].z;
					point_.r = (unsigned char)(cloud_->points[i].intensity);
					point_.g = 255;
					//point_.g = 0;
					point_.b = 0;
					cloud_add->push_back(point_);
				}
			}

			*cloud_sum += *cloud_add;

			index_PC_now++;
			if (index_PC_now == trajectory_vec_vec.size()) b_break = true;

		}
		else if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1)
		{
			cout << "ESCAPE pressed" << endl;

			b_escaped = true;
			break;
		}

		//if (cloud_sum->size() != 0) {
		//	ShowPcdFile(cloud_sum);
		//}

		if (b_break) break;
	}

	//output pcd
	if (b_break)
	{
		string filename_ = "map_chara.pcd";
		double th_VGF;
		//th_VGF = 0.01;
		th_VGF = 0.05;
		//th_VGF = 0.1;
		//th_VGF = 0.2;
		VGFilter_RGB.setInputCloud(cloud_sum);
		VGFilter_RGB.setLeafSize(th_VGF, th_VGF, th_VGF);
		VGFilter_RGB.filter(*cloud_sum);
		pcl::io::savePCDFile<pcl::PointXYZRGB>(foldername_ + "/" + filename_, *cloud_sum);
		cout << "saved: " << filename_ << endl;
		cout << endl;
	}

	//m_viewer->close();

}

void CVelodyneInterface::ShowOnlySequent(string foldername_)
{
	//foldername_ = "../savedfolder/20200119/PointCloud";
	//foldername_ = "\../savedfolder/naraha summer/sequent";
	CTimeString time_;
	vector<string> filenames_;
	bool b_nir = false;
	bool b_mix = false;
	//time_.getFileNames_extension(foldername_, filenames_, ".pcd");
	time_.getFileNames_extension(foldername_, filenames_, "nir.pcd"); b_nir = true;
	//time_.getFileNames_extension(foldername_, filenames_, "chara.pcd"); b_mix = true;
	//time_.getFileNames_extension(foldername_, filenames_, ".pcd"); b_mix = true;


	pcl::PointCloud<PointType>::Ptr cloud_(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());

	Eigen::Affine3f Trans_;
	Eigen::Matrix4d HM_free = Eigen::Matrix4d::Identity();

	bool b_first = true;
	int index_ = 0;

	cout << "file number:" << filenames_.size() << endl;

	while (!m_viewer->wasStopped()) {
		//change PointCloud
		short key_num = GetAsyncKeyState(VK_SPACE);
		if ((key_num & 1) == 1 || b_first) {

			if (index_ == filenames_.size())
			{
				cout << "index over" << endl;
				break;
			}

			cout << "reading:" << filenames_[index_] << endl;
			pcl::io::loadPCDFile(foldername_ + "/" + filenames_[index_], *cloud_);
			cout << "showing:" << filenames_[index_] << endl;

			////show max and min
			//float max_, min_;
			//max_ = 0.;
			//min_ = 255.;
			//for (int i = 0; i < cloud_->size(); i++)
			//{
			//	if (max_ < cloud_->points[i].intensity) max_ = cloud_->points[i].intensity;
			//	if (min_ > cloud_->points[i].intensity) min_ = cloud_->points[i].intensity;
			//}
			//cout << "max_ = " << max_ << endl;
			//cout << "min_ = " << min_ << endl;

			//range
			if (b_nir)
			{
				bool b_modify = false;
				b_modify = true;
				//turn pitch(camera axis)
				double pitch_init;
				pitch_init = 22. * M_PI / 180.;
				HM_free = Eigen::Matrix4d::Identity();
				HM_free = calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., pitch_init, 0.);
				Trans_ = Eigen::Affine3f::Identity();
				Trans_ = calcAffine3fFromHomogeneousMatrix(HM_free);
				pcl::transformPointCloud(*cloud_, *cloud_, Trans_);

				double th_x, th_z;
				//th_x = 3.;
				//th_z = -1.2;


				//th_x = 10.;
				th_x = 4.;
				th_z = -100.;


				//th_z = -0.4;
				cloud_temp->clear();
				pcl::copyPointCloud(*cloud_, *cloud_temp);
				cloud_->clear();
				for (int i = 0; i < cloud_temp->size(); i++)
				{
					PointType point_ = cloud_temp->points[i];
					if (point_.x > th_x && b_modify) continue;
					if (point_.z < th_z && b_modify) continue;

					cloud_->push_back(point_);
				}
			}

			//else if (b_mix)
			//{
			//	double max_ = 0.;
			//	for (int i = 0; i < cloud_->size(); i++)
			//		if (max_ > cloud_->points[i].intensity) max_ = cloud_->points[i].intensity;

			//	for (int i = 0; i < cloud_->size(); i++)
			//	{
			//		pcl::PointXYZRGB point_;
			//		point_.x = cloud_->points[i].x;
			//		point_.y = cloud_->points[i].y;
			//		point_.z = cloud_->points[i].z;
			//		point_.r = cloud_->points[i].intensity;
			//		//point_.r = (unsigned char)(cloud_->points[i].intensity / max_);
			//		point_.g = 255;
			//		cloud_add->push_back(point_);
			//	}

			//	point_.r = (unsigned char)(cloud_->points[i].intensity / max_);

			//}

			index_++;

			if (b_first) b_first = false;
		}
		ShowPcdFile(cloud_);

		//escape
		short key_num_esc = GetAsyncKeyState(VK_ESCAPE);
		if ((key_num_esc & 1) == 1) {
			cout << "toggled!" << endl;
			break;
		}
	}

	cout << "finished" << endl;

	m_viewer->close();

}

void CVelodyneInterface::getOnlyShow3(vector<CRobotState> state_arg)
{
	//for (int i = 0; i < 181; i++) {
	//	cout << "RangeData[" << i - 90 << "] = " << vector_arg[i] << endl;

	//}
	for (int i = 0; i < state_arg.size(); i++) {
		cout << "StateDataX[" << (int)(i - (state_arg.size() - 1) / 2) << "] = " << state_arg[i].x << endl;
		cout << "StateDataY[" << (int)(i - (state_arg.size() - 1) / 2) << "] = " << state_arg[i].y << endl << endl;;

	}

	getchar();
}

void CVelodyneInterface::getOnlyShow2(vector<double> vector_arg) 
{
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

void CVelodyneInterface::ShowPcdFile(pcl::PointCloud<PointType>::Ptr p_cloud)
{

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

void CVelodyneInterface::TimerWrite()
{

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



