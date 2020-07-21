#include"TimeString.h"

string CTimeString::getTimeString() 
{
	int i_year, i_month, i_day, i_hour, i_minute, i_second, i_milliseconds;
	setTime(i_year, i_month, i_day, i_hour, i_minute, i_second, i_milliseconds);

	string s_year, s_month, s_day, s_hour, s_minute, s_second, s_milliseconds,s_time;
	s_year = to_string(i_year);
	s_month = to_string(i_month);						//size 2
	if (s_month.size() < 2) s_month = "0" + s_month;
	s_day = to_string(i_day);							//size 2
	if (s_day.size() < 2) s_day = "0" + s_day;
	s_hour = to_string(i_hour);							//size 2
	if (s_hour.size() < 2) s_hour = "0" + s_hour;
	s_minute = to_string(i_minute);						//size 2
	if (s_minute.size() < 2) s_minute = "0" + s_minute;
	s_second = to_string(i_second);						//size 2
	if (s_second.size() < 2) s_second = "0" + s_second;
	s_milliseconds = to_string(i_milliseconds);			//size 3
	if (s_milliseconds.size() < 3) s_milliseconds = "0" + s_milliseconds;
	if (s_milliseconds.size() < 3) s_milliseconds = "0" + s_milliseconds;
	//s_milliseconds.erase(s_milliseconds.begin() + 1, s_milliseconds.begin() + 3);

	s_time = s_year + s_month + s_day + "_" + s_hour + s_minute + "_" + s_second + "_" + s_milliseconds;
	return s_time;
}

string CTimeString::getTimeElapsefrom2Strings(string s_former, string s_latter) 
{

	//20190504_2130_33_119

	//former
	int hour_f, minute_f, second_f, millisecond_f;
	getTimeValueFromString(s_former, hour_f, minute_f, second_f, millisecond_f);

	//latter
	int hour_l, minute_l, second_l, millisecond_l;
	getTimeValueFromString(s_latter, hour_l, minute_l, second_l, millisecond_l);

	int sum_millisecond_f, sum_millisecond_l;
	sum_millisecond_f = sum_millisecond_l = 0;
	sum_millisecond_f = millisecond_f + 1000 * (second_f + 60 * (minute_f + 60 * hour_f));
	sum_millisecond_l = millisecond_l + 1000 * (second_l + 60 * (minute_l + 60 * hour_l));

	int error_sum = 0;
	if (sum_millisecond_f == sum_millisecond_l)
		return "00_00_00_000";
	else
		error_sum = abs(sum_millisecond_f - sum_millisecond_l);

	int hour_e, minute_e, second_e, millisecond_e;

	//millisec
	millisecond_e = error_sum % 1000;	//999 millis + 121 *1000millis + 121*60 *1000millis
	error_sum -= millisecond_e;			//121 *1000millis + 121*60 *1000millis

	//second
	error_sum /= 1000;					//121s + 121*60s
	second_e = error_sum % 60;			//1s
	error_sum -= second_e;				//120s + 121*60s

	//minute
	error_sum /= 60;					//2min + 121min
	minute_e = error_sum % 60;			//3min
	error_sum -= minute_e;				//120min

	//hour
	error_sum /= 60;					//2hour
	hour_e = error_sum % 24;			//2hour
	error_sum -= hour_e;				//0*24hour

	string s_hour, s_minute, s_second, s_milliseconds, s_output;

	s_hour = to_string(hour_e);
	if (s_hour.size() < 2) s_hour = "0" + s_hour;

	s_minute = to_string(minute_e);						//size 2
	if (s_minute.size() < 2) s_minute = "0" + s_minute;

	s_second = to_string(second_e);						//size 2
	if (s_second.size() < 2) s_second = "0" + s_second;

	s_milliseconds = to_string(millisecond_e);			//size 3
	if (s_milliseconds.size() < 3) s_milliseconds = "0" + s_milliseconds;
	if (s_milliseconds.size() < 3) s_milliseconds = "0" + s_milliseconds;

	s_output = s_hour + "_" + s_minute + "_" + s_second + "_" + s_milliseconds;

	return s_output;
}

void CTimeString::getTimeValueFromString(string string_, int &i_hour, int &i_minute, int &i_second, int &i_millisecond)
{
	vector<int> find_vec = find_all(string_, "_");
	string s_hour, s_minute, s_second, s_millisecond;
	s_hour = string_.substr(find_vec[0] + 1, 2);
	s_minute = string_.substr(find_vec[0] + 3, 2);
	s_second = string_.substr(find_vec[1] + 1, 2);
	s_millisecond = string_.substr(find_vec[2] + 1, 3);
	string s_day;
	s_day = string_.substr(find_vec[0] - 2, 2);
	i_hour = stoi(s_hour) + 24 * stoi(s_day);
	i_minute = stoi(s_minute);
	i_second = stoi(s_second);
	i_millisecond = stoi(s_millisecond);
}

void CTimeString::setTime(int& i_year, int& i_month, int& i_day, int& i_hour, int& i_minute, int& i_second, int& i_milliseconds)
{
	SYSTEMTIME st;
	GetSystemTime(&st);
	i_year = st.wYear;
	i_month = st.wMonth;
	i_day = st.wDay;
	i_hour = st.wHour+9;
	i_minute = st.wMinute;
	i_second = st.wSecond;
	i_milliseconds = st.wMilliseconds;
	if (i_hour >= 24) {
		i_hour -= 24;
		i_day++;
	}
}

//https://www.sejuku.net/blog/49318
std::vector<int> CTimeString::find_all(const std::string str, const std::string subStr)
{
	std::vector<int> result;

	int subStrSize = subStr.size();
	int pos = str.find(subStr);

	while (pos != std::string::npos) {
		result.push_back(pos);
		pos = str.find(subStr, pos + subStrSize);
	}

	return result;
}

 bool CTimeString::getFileNames(std::string folderPath, std::vector<std::string> &file_names, bool b_cout, bool b_getDir, bool b_check)
{
	 //if (b_check)
		// if (!getDirectoryExistance(folderPath))
		//	 return false;
	 if (b_check)
		 if (!getDirectoryExistance_detail(folderPath,true))
			 return false;

	//only work in Multibyte Character Set (not in Unicode)
	//https://qiita.com/tes2840/items/8d295b1caaf10eaf33ad
	//HANDLE hFind;
	//WIN32_FIND_DATA win32fd;
	//std::string search_name = folderPath + "\\*";
	//hFind = FindFirstFile(search_name.c_str(), &win32fd);
	//if (hFind == INVALID_HANDLE_VALUE) {
	//	throw std::runtime_error("file not found");
	//	return false;
	//}
	//do
	//{
	//	if (win32fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
	//		cout << win32fd.cFileName << "(directory)" << endl;
	//	}
	//	else {
	//		file_names.push_back(win32fd.cFileName);
	//		//cout << file_names.back() << endl;
	//	}
	//} while (FindNextFile(hFind, &win32fd));
	//FindClose(hFind);

	//work in Multibyte Character Set and Unicode
	//https://qiita.com/takamon9/items/a9f1fccea740f2b79991
	//https://qiita.com/episteme/items/0e3c2ee8a8c03780f01e
	sys_ns::path fPath(folderPath);
	sys_ns::directory_iterator itr(fPath), end;
	std::error_code err;
	file_names.clear();
	for (itr; itr != end; itr.increment(err)) {
		if (err != std::errc::operation_not_permitted) {
			auto entry = *itr; // auto = std::experimental::filesystem::v1::path
			string path_name = entry.path().generic_string();
			//cout << "substr:dir:  = " << folderPath << endl;
			//cout << "substr:file: = " << path_name << endl;
			string path_relative = path_name.substr(folderPath.size() + 1, 
				path_name.size() - folderPath.size() - 1);//startposition,size
			//cout << "substr:path_relative = " << path_relative << endl;
			if(b_cout)	cout << "getFileNames: ";
			if (sys_ns::is_directory(entry.path()))
			{
				if (b_getDir) file_names.push_back(path_relative);
				if (b_cout)	cout << path_relative << "(directory)" << endl;
			}
			else
			{
				file_names.push_back(path_relative); // add to the vector.
				if (b_cout) cout << path_relative << endl;
			}
		}
		else break;
	}
	return true;
}

bool CTimeString::getFileNames_extension(std::string folderPath, std::vector<std::string> &file_names, string s_extension)
{
	//https://www.sejuku.net/blog/49318
	vector<string> filenames_;
	bool b_success = getFileNames(folderPath, filenames_, false, false, true);
	for (int i = 0; i < filenames_.size(); i++)
	{
		int i_find = filenames_[i].find(s_extension);
		if (i_find == std::string::npos) continue;
		file_names.push_back(filenames_[i]);
		cout << file_names.back() << endl;
	}
	return b_success;
}

int CTimeString::getTimeElapsefrom2Strings_millisec(string s_former, string s_latter)
{
	string t_diff = getTimeElapsefrom2Strings(s_former, s_latter);
	//XX_XX_XX_XXX, hour_min_second_millisec

	int i_value;

	vector<int> find_vec = find_all(t_diff, "_");		//realy, not need
	string s_hour, s_minute, s_second, s_millisecond;

	s_hour = t_diff.substr(0, 2);
	s_minute = t_diff.substr(find_vec[0] + 1, 2);
	s_second = t_diff.substr(find_vec[1] + 1, 2);
	s_millisecond = t_diff.substr(find_vec[2] + 1, 3);

	i_value =
		stoi(s_millisecond)
		+ stoi(s_second) * 1000
		+ stoi(s_minute) * 1000 * 60
		+ stoi(s_hour) * 1000 * 60 * 60;

	return i_value;
}

//if upper directory path is wrong, return false
bool CTimeString::getDirectoryExistance(string foder_Path)
{
	bool b_exist = false;
	vector<int> find_vec = find_all(foder_Path, "/");
	if (find_vec.size() == 0)
		throw std::runtime_error("ERROR(CTimeString::getDirectoryExistance): folderPath not contains / \n");

	string foder_Path_upper = foder_Path.substr(0, find_vec.back());
	//cout << "checking: " << foder_Path << endl;

	vector<string> s_vec;
	CTimeString::getFileNames(foder_Path_upper, s_vec, false, true, false);

	string foder_Path_relative = foder_Path.substr(find_vec.back()+1,
		foder_Path.size() - foder_Path_upper.size() - 1);//startposition,size
	//"xxx/yyyy" -> "8-3-1=3"
	//cout << "foder_Path_relative: " << foder_Path_relative << endl;

	//https://www.sejuku.net/blog/62561
	for (int i = 0; i < s_vec.size(); i++)
		if (s_vec[i] == foder_Path_relative) b_exist = true;

	//if (!b_exist) cout << "NOT FOUND: " << foder_Path << endl;
	return b_exist;
}

bool CTimeString::getDirectoryExistance_detail(string foder_Path,bool b_first)
{
	bool b_exist = false;

	vector<int> find_vec = find_all(foder_Path, "/");
	if (find_vec.size() == 0)
		throw std::runtime_error("ERROR(CTimeString::getDirectoryExistance): folderPath not contains / \n");

	if (b_first)
	{
		if (CTimeString::getDirectoryExistance(foder_Path)) b_exist = true;
	}
	else
	{
		if (CTimeString::getDirectoryExistance(foder_Path)) 
		{
			b_exist = true;
			cout << "FOUND:     " << foder_Path << endl;
		}
	}

	if (!b_exist)
	{
		string foder_Path_upper = foder_Path.substr(0, find_vec.back());
		if (!b_exist) cout << "NOT FOUND: " << foder_Path << endl;
		CTimeString::getDirectoryExistance_detail(foder_Path_upper, false);
	}

	return b_exist;
}

vector<vector<string>> CTimeString::getVecVecFromCSV_string(string filename_, string key_token)
{
	//double,float, int
	//https://qiita.com/hal1437/items/b6deb22a88c76eeaf90c
	//https://docs.oracle.com/cd/E19957-01/805-7887/6j7dsdhfl/index.html
	//https://pknight.hatenablog.com/entry/20090826/1251303641
	ifstream ifs_(filename_);
	string str_;
	int f_cnt = 0;
	vector<vector<string>> all_observation_vec_vec;
	if (ifs_.fail()) cout << "Error: file could not be read." << endl;
	else
	{
		while (getline(ifs_, str_)) {//readed to string from file

			vector<string> one_observation_vec;
			vector<int> find_vec = CTimeString::find_all(str_, key_token);
			if (find_vec.size() == 0)
				one_observation_vec.push_back(str_);
			else
			{
				one_observation_vec.push_back(str_.substr(0, find_vec[0]));
				int s_pos = 0;
				while (s_pos < find_vec.size() - 1)
				{
					one_observation_vec.push_back(
						str_.substr(find_vec[s_pos] + 1, find_vec[s_pos + 1] - (find_vec[s_pos] + 1)));
					s_pos++;
				}
				one_observation_vec.push_back(str_.substr(find_vec[s_pos] + 1, str_.size() - (find_vec[s_pos] + 1)));

			}
			all_observation_vec_vec.push_back(one_observation_vec);
		}
		ifs_.close();
	}
	return all_observation_vec_vec;
}

vector<vector<double>> CTimeString::getVecVecFromCSV(string filename_)
{
	vector<vector<double>> data_vec_vec;

	vector<vector<string>> data_vec_vec_string;
	data_vec_vec_string = CTimeString::getVecVecFromCSV_string(filename_);

	for (int j = 0; j < data_vec_vec_string.size(); j++)
	{
		vector<double> data_vec;
		for (int i = 0; i < data_vec_vec_string[j].size(); i++)
		{
			double value_ = stod(data_vec_vec_string[j][i]);
			data_vec.push_back(value_);
		}
		data_vec_vec.push_back(data_vec);
	}

	return data_vec_vec;
}

void CTimeString::copyfile(string filename_from, string filename_to)
{
	//https://hakase0274.hatenablog.com/entry/2019/09/03/235304
	sys_ns::copy(filename_from, filename_to);
	//sys_ns::copy_file
	//std::filesystem::copy();
}

void CTimeString::deletefile(string filename_delete)
{
	sys_ns::remove_all(filename_delete);
}

void CTimeString::makenewfolder(string dir, string newfoldername)
{
	sys_ns::create_directory(dir + "/" + newfoldername);
}

bool CTimeString::getFileNames_folder(std::string folderPath, std::vector<std::string> &file_names)
{
	vector<string> filenames_;
	bool b_success = getFileNames(folderPath, filenames_, false, true, true);
	for (int i = 0; i < filenames_.size(); i++)
	{
		if (!sys_ns::is_directory(folderPath + "/" + filenames_[i])) continue;
		file_names.push_back(filenames_[i]);
	}
	return b_success;
}

vector<string> CTimeString::inputSomeString()
{
	vector<string> s_vec;

	string s_input;
	//cout << "input value separated by spaces" << endl;
	//https://programming.pc-note.net/cpp/iostream.html#clear
	cin.ignore(1024, '\n');
	std::getline(std::cin, s_input);
	if (s_input.size() == 0)
	{
		//cout << "ERROR: no file inputed." << endl;
		return s_vec;
	}
	{
		vector<int> i_space_vec_temp;
		i_space_vec_temp = find_all(s_input, " ");
		if (i_space_vec_temp.size() == 0)
		{
			s_vec.push_back(s_input);
			return s_vec;
		}
	}

	//erase front space
	while (1)
	{
		vector<int> i_space_vec_temp;
		i_space_vec_temp = find_all(s_input, " ");
		if (i_space_vec_temp[0] == 0) s_input.erase(s_input.begin());
		else break;
	}
	//erase end space
	while(1)
	{
		vector<int> i_space_vec_temp;
		i_space_vec_temp = find_all(s_input, " ");
		if (i_space_vec_temp.back() == s_input.size() - 1) s_input.pop_back();
		else break;
	}
	//erase rest space
	{
		//erase http://farma-11.hatenablog.com/entry/2018/01/18/094729
		vector<int> i_space_vec_temp;
		i_space_vec_temp = find_all(s_input, " ");
		vector<int> i_space_erasePos_vec;
		for (int i = 0; i < i_space_vec_temp.size(); i++)
		{
			if (i == 0) continue;
			if (i_space_vec_temp[i] == i_space_vec_temp[i - 1] + 1)
				i_space_erasePos_vec.push_back(i_space_vec_temp[i]);	//11 111 _1111 __11111 
		}
		//cout << "s_input:" << s_input << endl;
		//use "back-for" because s_input.size() is decreasing.
		for (int i = i_space_erasePos_vec.size() - 1; i >= 0; i--)
		{
			//erase char at i
			s_input.erase(s_input.begin() + i_space_erasePos_vec[i] - 1);
		}
		//cout << "s_input(new):" << s_input << endl;
	}

	//compute output
	vector<int> i_space_vec;
	i_space_vec = find_all(s_input, " ");
	for (int i = 0; i < i_space_vec.size(); i++)
	{
		if (i == 0) s_vec.push_back(s_input.substr(0, i_space_vec[i]));	//11_111 1111 11111
		else s_vec.push_back(s_input.substr(i_space_vec[i - 1] + 1,
				i_space_vec[i] - (i_space_vec[i - 1] + 0)));		    //11_111_1111 11111
	}
	s_vec.push_back(s_input.substr(	i_space_vec.back() + 1,
		s_input.size() - (i_space_vec.back() + 1)));					//11 111 1111_11111
	//back() https://kaworu.jpn.org/cpp/std::vector
	//for (int i = 0; i < s_vec.size(); i++)
	//	cout << "i:" << i << " " << s_vec[i] << endl;
	return s_vec;
}

vector<string> CTimeString::inputSomeString_fromCSV(string s_filename)
{
	vector<string> s_vec_output;

	vector<vector<string>> s_vecvec = getVecVecFromCSV_string(s_filename);
	for (int j = 0; j < s_vecvec.size(); j++)
		for (int i = 0; i < s_vecvec[j].size(); i++)
			s_vec_output.push_back(s_vecvec[j][i]);

	return s_vec_output;
}


void CTimeString::showParameter(vector<float> parameter_vec, vector<string> name_vec, int i_frame_show)
{
	if (i_frame_show == -1) i_frame_show = name_vec.size() - 1;

	if (parameter_vec.size() != parameter_vec.size())
	{
		throw std::runtime_error("ERROR(CTimeString::changeParameter): vector size is different");
	}

	//count longest string
	int size_string_longest = 0;
	for (int i = 0; i < name_vec.size(); i++)
	{
		if (size_string_longest < name_vec[i].size()) size_string_longest = name_vec[i].size();
	}

	//show parameter
	for (int i = 0; i < i_frame_show + 1; i++)
	{
		string s_show = name_vec[i];
		s_show += ":";
		while (1)
		{
			if (s_show.size() < size_string_longest + 1) s_show += " ";
			else break;
		}
		cout << i << ": " << s_show << "  " << parameter_vec[i] << endl;
	}

}

void CTimeString::changeParameter(vector<float> &parameter_vec, vector<string> name_vec)
{
	if(parameter_vec.size() != name_vec.size())
	{
		throw std::runtime_error("ERROR(CTimeString::changeParameter): vector size is different");
	}

	////count longest string
	//int size_string_longest = 0;
	//for (int i = 0; i < name_vec.size(); i++)
	//{
	//	if (size_string_longest < name_vec[i].size()) size_string_longest = name_vec[i].size();
	//}

	//show parameter
	cout << "Parameter list" << endl;
	showParameter(parameter_vec, name_vec);

	//change parameter
	bool b_parameter_changed = false;
	while (1)
	{

		int i_change = -1;
		cout << "select parameters to change  (ESCAPE by typing single 0 with no value )" << endl;
		cout << "->XX(parameter index) YY(value)" << endl;
		vector<string> s_input_vec;
		s_input_vec.clear();
		s_input_vec = CTimeString::inputSomeString();
		cout << "s_input_vec.size():" << s_input_vec.size() << endl;

		if (!(s_input_vec.size() == 1 || s_input_vec.size() == 2)) continue;

		if (s_input_vec.size() == 1)
		{
			if (stoi(s_input_vec[0]) == 0) break;
			else continue;
		}

		//s_input_vec.size() == 2
		i_change = stoi(s_input_vec[0]);
		if (!(0 <= i_change && i_change < name_vec.size() - 1)) continue;

		float value_ = stof(s_input_vec[1]);
		b_parameter_changed = true;

		//change value
		parameter_vec[i_change] = value_;
		cout << name_vec[i_change] << ": " << parameter_vec[i_change] << endl;
		cout << "parameter changed" << endl;
		cout << endl;
	}

	if (b_parameter_changed)
	{
		cout << endl;
		cout << "Parameter list (new)" << endl;
		showParameter(parameter_vec, name_vec);
	}

}

void CTimeString::changeParameter(vector<float> &parameter_vec, vector<string> name_vec, string filename_,
	int row_small, int row_big, int col_)
{
	cout << "Parameter list" << endl;
	showParameter(parameter_vec, name_vec);

	cout << "press 1 and Enter if you have closed file" << endl;
	{
		int aa;
		cin >> aa;
	}

	vector<vector<string>> s_vec_vec_temp;
	s_vec_vec_temp = getVecVecFromCSV_string(filename_);

	if (!(0 <= row_big && row_big < s_vec_vec_temp.size())) row_big = s_vec_vec_temp.size() - 1;
	if (!(0 <= row_small && row_small < row_big)) row_small = 0;
	if (!(0 <= col_ && col_ < s_vec_vec_temp[0].size())) col_ = s_vec_vec_temp[0].size() - 1;

	vector<float> parameter_vec_new;
	for (int j = row_small; j < row_big + 1; j++)
	{
		float value_;
		if (s_vec_vec_temp[j][col_].size() == 0) value_ = 0.;
		else value_ = stof(s_vec_vec_temp[j][col_]);
		parameter_vec_new.push_back(value_);
	}

	if(parameter_vec.size() != parameter_vec_new.size())
		throw std::runtime_error("ERROR(CTimeString::changeParameter): parameter size invalid");

	parameter_vec = parameter_vec_new;

	cout << "Parameter list (new)" << endl;
	showParameter(parameter_vec, name_vec);

}

void CTimeString::changeParameter_2dimension(vector<vector<float>> &parameter_vec_vec, vector<string> name_vec, vector<float> parameter_vec_init)
{

	if (parameter_vec_init.size() != name_vec.size())
		throw std::runtime_error("ERROR(CTimeString::changeParameter): vector size is different");

	parameter_vec_vec.clear();
	parameter_vec_vec.resize(parameter_vec_init.size());

	//show parameter
	cout << "Parameter list" << endl;
	CTimeString::showParameter(parameter_vec_init, name_vec);

	while (1)
	{
		int i_change = -1;
		cout << "input parameters to change  (ESCAPE by typing single 0 with no value )" << endl;
		cout << "->XX(parameter index) AA(value) BB(value) CC(value) ..." << endl;
		vector<string> s_input_vec;
		s_input_vec.clear();
		s_input_vec = CTimeString::inputSomeString();
		cout << "s_input_vec.size():" << s_input_vec.size() << endl;

		if (s_input_vec.size() == 1)
		{
			if (stoi(s_input_vec[0]) == 0) break;
			else continue;
		}

		i_change = stoi(s_input_vec[0]);
		if (!(0 <= i_change && i_change < name_vec.size())) continue;

		vector<float> value_vec;
		for (int j = 1; j < s_input_vec.size(); j++)
			value_vec.push_back(stof(s_input_vec[j]));

		cout << "value_vec" << endl;
		for (int i = 0; i < value_vec.size(); i++)
			cout << "i:" << i << " " << value_vec[i] << endl;

		//change value
		cout << "parameter inputed" << endl;
		parameter_vec_vec[i_change] = value_vec;
		cout << name_vec[i_change] << ": ";
		for (int j = 0; j < parameter_vec_vec[i_change].size(); j++)
			cout << parameter_vec_vec[i_change][j] << " ";
		cout << endl;
	}

	//fill blank parameter
	for (int j = 0; j < parameter_vec_vec.size(); j++)
	{
		if (parameter_vec_vec[j].size() == 0)
			parameter_vec_vec[j].push_back(parameter_vec_init[j]);
	}

	//count longest string for showing
	int size_string_longest = 0;
	for (int i = 0; i < name_vec.size(); i++)
		if (size_string_longest < name_vec[i].size()) size_string_longest = name_vec[i].size();
	//show parameter
	cout << "Parameter list (vector)" << endl;
	for (int j = 0; j < parameter_vec_vec.size(); j++)
	{
		string s_show = name_vec[j];
		s_show += ":";
		while (1)
		{
			if (s_show.size() < size_string_longest + 1) s_show += " ";
			else break;
		}
		cout << j << ": " << s_show;
		for (int i = 0; i < parameter_vec_vec[j].size(); i++)
			cout << "  " << parameter_vec_vec[j][i];
		cout << endl;
	}
	cout << endl;

}

void CTimeString::calcParameterPattern(vector<vector<float>> &pattern_vec_vec, vector<vector<float>> parameter_vec_vec)
{
	//make pattern_vec_vec
	int num_pattern;
	num_pattern = 1;
	for (int j = 0; j < parameter_vec_vec.size(); j++)
		num_pattern *= parameter_vec_vec[j].size();
	pattern_vec_vec.resize(num_pattern);
	for (int j = 0; j < num_pattern; j++)
		pattern_vec_vec[j].resize(parameter_vec_vec.size());

	//insert to pattern_vec_vec
	for (int j = 0; j < num_pattern; j++)
	{
		int i_devide = num_pattern;
		int i_residual = j;
		for (int i = 0; i < parameter_vec_vec.size(); i++)
		{
			int idx;
			i_devide /= parameter_vec_vec[i].size();
			idx = i_residual / i_devide;
			i_residual %= i_devide;
			pattern_vec_vec[j][i] = parameter_vec_vec[i][idx];
		}
	}

	removeSameParameter(parameter_vec_vec);

	cout << "show pattern" << endl;
	for (int j = 0; j < pattern_vec_vec.size(); j++)
	{
		cout << j << ":";
		for (int i = 0; i < pattern_vec_vec[j].size(); i++)
		{
			string s_value;
			s_value = to_string(pattern_vec_vec[j][i]);
			if (s_value.size() < 4) s_value = " " + s_value;
			if (s_value.size() < 4) s_value = " " + s_value;
			if (s_value.size() < 4) s_value = " " + s_value;
			cout << "  " << s_value;

		}
		cout << endl;
	}
	cout << endl;

}

void CTimeString::changeParameter_2dimension(vector<vector<float>> &parameter_vec_vec, vector<string> name_vec, vector<float> parameter_vec_init,
	string filename_, int row_small, int col_small, int row_big, int col_big)
{
	cout << "Parameter list" << endl;
	showParameter(parameter_vec_init, name_vec);

	parameter_vec_vec.clear();

	//cout << "press 1 and Enter if you have closed file" << endl;
	//{
	//	int aa;
	//	cin >> aa;
	//}

	vector<vector<string>> s_vec_vec_temp;
	s_vec_vec_temp = getVecVecFromCSV_string(filename_);

	if (!(0 <= row_big && row_big < s_vec_vec_temp.size())) row_big = s_vec_vec_temp.size() - 1;
	if (!(0 <= row_small && row_small < row_big)) row_small = 0;
	if (!(0 <= col_big && col_big < s_vec_vec_temp[0].size())) col_big = s_vec_vec_temp[0].size() - 1;
	if (!(0 <= col_small && col_small < col_big)) col_small = 0;

	vector<vector<float>> parameter_vec_vec_new;
	for (int j = row_small; j < row_big + 1; j++)
	{
		vector<float> parameter_vec_new;
		for (int i = col_small; i < col_big + 1; i++)
		{
			float value_;
			if (s_vec_vec_temp[j][i].size() == 0) continue;
			else value_ = stof(s_vec_vec_temp[j][i]);
			parameter_vec_new.push_back(value_);
		}
		parameter_vec_vec_new.push_back(parameter_vec_new);
	}

	if (parameter_vec_init.size() != parameter_vec_vec_new.size())
		throw std::runtime_error("ERROR(CTimeString::changeParameter): parameter size invalid");

	parameter_vec_vec = parameter_vec_vec_new;

	removeSameParameter(parameter_vec_vec);

	//count longest string for showing
	int size_string_longest = 0;
	for (int i = 0; i < name_vec.size(); i++)
		if (size_string_longest < name_vec[i].size()) size_string_longest = name_vec[i].size();
	//show parameter
	cout << "Parameter list (vector)" << endl;
	for (int j = 0; j < parameter_vec_vec.size(); j++)
	{
		string s_show = name_vec[j];
		s_show += ":";
		while (1)
		{
			if (s_show.size() < size_string_longest + 1) s_show += " ";
			else break;
		}
		cout << j << ": " << s_show;
		for (int i = 0; i < parameter_vec_vec[j].size(); i++)
			cout << "  " << parameter_vec_vec[j][i];
		cout << endl;
	}
	cout << endl;

}

void CTimeString::removeSameParameter(vector<vector<float>> &parameter_vec_vec)
{

	for (int j = 0; j < parameter_vec_vec.size(); j++)
	{
		vector<float> parameter_vec = parameter_vec_vec[j];
		vector<float> parameter_vec_new;
		//sort
		sort(parameter_vec.begin(), parameter_vec.end());
		//remove same value
		float value_last;
		for (int i = 0; i < parameter_vec.size(); i++)
		{
			if (i == 0)
			{
				parameter_vec_new.push_back(parameter_vec[i]);
			}
			else
			{
				if (value_last != parameter_vec[i])
					parameter_vec_new.push_back(parameter_vec[i]);
			}
			value_last = parameter_vec[i];
		}
		parameter_vec_vec[j] = parameter_vec_new;
	}
}

void CTimeString::sortStringVector2d(vector<vector<string>> &s_vecvec, int index_arg)
{
	vector<pair<int, int>> frame_pair_vec;
	for (int j = 0; j < s_vecvec.size(); j++)
		frame_pair_vec.push_back(make_pair(stoi(s_vecvec[j][index_arg]), j));
	for (int i = 0; i < frame_pair_vec.size(); i++)
	{
		for (int j = frame_pair_vec.size() - 1; j > i; j--)
		{
			if (frame_pair_vec[j].first < frame_pair_vec[j - 1].first)
				swap(frame_pair_vec[j], frame_pair_vec[j - 1]);
		}
	}
	vector<vector<string>> s_input_vecvec_new;
	for (int j = 0; j < frame_pair_vec.size(); j++)
	{
		vector<string> s_input_vec_new;
		s_input_vecvec_new.push_back(s_vecvec[frame_pair_vec[j].second]);
	}
	s_vecvec.clear();
	s_vecvec = s_input_vecvec_new;
}

void CTimeString::sortStringVector2d_2ingredient(vector<vector<string>> &s_vecvec, int ing_large, int ing_small)
{
	////show
	//for (int j = 0; j < s_vecvec.size(); j++)
	//{
	//	for (int i = 0; i < s_vecvec[j].size(); i++)
	//		cout << s_vecvec[j][i] << "  ";
	//	cout << endl;
	//}

	//large sort
	CTimeString::sortStringVector2d(s_vecvec, ing_large);

	//small sort while sustaining large sort
	vector<vector<string>> s_vecvec_new;
	vector<vector<string>> s_input_vecvec_1unit;
	int i_tgt = 0;
	int idx = 0;
	while (1)
	{
		bool b_end_inFrame = false;
		if (i_tgt == stoi(s_vecvec[idx][ing_large]))
			s_input_vecvec_1unit.push_back(s_vecvec[idx]);

		if (s_vecvec.size() == idx + 1)
			b_end_inFrame = true;

		else if (i_tgt != stoi(s_vecvec[idx + 1][ing_large]))
			b_end_inFrame = true;

		if (b_end_inFrame)
		{
			CTimeString::sortStringVector2d(s_input_vecvec_1unit, ing_small);
			for (int j = 0; j < s_input_vecvec_1unit.size(); j++)
				s_vecvec_new.push_back(s_input_vecvec_1unit[j]);
			s_input_vecvec_1unit.clear();

			i_tgt++;
			//if (frame_end == i_tgt) break;
			if (s_vecvec.size() == idx + 1) break;

			continue;

		}
		idx++;
		//if (s_vecvec.size() == idx) break;
	}

	s_vecvec.clear();
	s_vecvec = s_vecvec_new;
}

