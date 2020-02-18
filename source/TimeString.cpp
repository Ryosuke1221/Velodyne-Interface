#include"TimeString.h"

string CTimeString::getTimeString() {
	setTime();
	string s_year, s_month, s_day, s_hour, s_minute, s_second, s_milliseconds,s_time;

	s_year = to_string(m_year);

	s_month = to_string(m_month);						//size 2
	if (s_month.size() < 2) s_month = "0" + s_month;

	s_day = to_string(m_day);							//size 2
	if (s_day.size() < 2) s_day = "0" + s_day;

	s_hour = to_string(m_hour);							//size 2
	if (s_hour.size() < 2) s_hour = "0" + s_hour;

	s_minute = to_string(m_minute);						//size 2
	if (s_minute.size() < 2) s_minute = "0" + s_minute;

	s_second = to_string(m_second);						//size 2
	if (s_second.size() < 2) s_second = "0" + s_second;

	s_milliseconds = to_string(m_milliseconds);			//size 3
	if (s_milliseconds.size() < 3) s_milliseconds = "0" + s_milliseconds;
	if (s_milliseconds.size() < 3) s_milliseconds = "0" + s_milliseconds;
	//s_milliseconds.erase(s_milliseconds.begin() + 1, s_milliseconds.begin() + 3);

	s_time = s_year + s_month + s_day + "_" + s_hour + s_minute + "_" + s_second + "_" + s_milliseconds;
	return s_time;
}

string CTimeString::getTimeElapsefrom2Strings(string s_former, string s_latter) {

	//20190504_2130_33_119

	//former
	int minute_f, second_f, millisecond_f;
	getTimeValueFromString(s_former, minute_f, second_f, millisecond_f);

	//latter
	int minute_l, second_l, millisecond_l;
	getTimeValueFromString(s_latter, minute_l, second_l, millisecond_l);

	int sum_millisecond_f, sum_millisecond_l;
	sum_millisecond_f = millisecond_f + 1000 * (second_f + 60 * minute_f);
	sum_millisecond_l = millisecond_l + 1000 * (second_l + 60 * minute_l);

	int error_sum_millisecond = 0;
	int minute_e, second_e, millisecond_e;
	if (sum_millisecond_f > sum_millisecond_l) {

		error_sum_millisecond = sum_millisecond_f - sum_millisecond_l;
	}
	else if (sum_millisecond_f < sum_millisecond_l) {

		error_sum_millisecond = sum_millisecond_l - sum_millisecond_f;
	}
	else {

		return "00_00_000";
	}

	millisecond_e = error_sum_millisecond % 1000;
	second_e = (int)((error_sum_millisecond - millisecond_e) / 1000) % 60;
	minute_e = (int)((error_sum_millisecond - millisecond_e) / 1000) / 60;

	string s_minute, s_second, s_milliseconds, s_output;

	s_minute = to_string(minute_e);						//size 2
	if (s_minute.size() < 2) s_minute = "0" + s_minute;

	s_second = to_string(second_e);						//size 2
	if (s_second.size() < 2) s_second = "0" + s_second;

	s_milliseconds = to_string(millisecond_e);			//size 3
	if (s_milliseconds.size() < 3) s_milliseconds = "0" + s_milliseconds;
	if (s_milliseconds.size() < 3) s_milliseconds = "0" + s_milliseconds;

	s_output = s_minute + "_" + s_second + "_" + s_milliseconds;

	return s_output;
}

string CTimeString::getTElapsefrom2S(string s_former, string s_latter){

	//former
	int hour_f, minute_f, second_f, millisecond_f;
	getTimeValueFromString(s_former, hour_f, minute_f, second_f, millisecond_f);

	//latter
	int hour_l, minute_l, second_l, millisecond_l;
	getTimeValueFromString(s_latter, hour_l, minute_l, second_l, millisecond_l);

	int sum_millisecond_f, sum_millisecond_l;
	sum_millisecond_f = millisecond_f + 1000 * (second_f + 60 * (minute_f + 60 * hour_f));
	sum_millisecond_l = millisecond_l + 1000 * (second_l + 60 * (minute_l + 60 * hour_l));

	int error_sum_millisecond = 0;
	int hour_e, minute_e, second_e, millisecond_e;
	if (sum_millisecond_f > sum_millisecond_l) {

		error_sum_millisecond = sum_millisecond_f - sum_millisecond_l;
	}
	else if (sum_millisecond_f < sum_millisecond_l) {

		error_sum_millisecond = sum_millisecond_l - sum_millisecond_f;
	}
	else {

		return "00_00_000";
	}

	int second_remain, minute_remain, hour_remain;

	millisecond_e = error_sum_millisecond % 1000;
	second_remain = (error_sum_millisecond - millisecond_e) / 1000;

	second_e = second_remain % 60;
	minute_remain = (second_remain - second_e) / 60;

	minute_e = minute_remain % 60;
	hour_remain = (minute_remain - minute_e) / 60;

	hour_e = hour_remain;

	string s_hour, s_minute, s_second, s_milliseconds, s_output;

	s_hour = to_string(hour_e);							//size 2
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


void CTimeString::getTimeValueFromString(string string_, int &i_hour, int &i_minute, int &i_second, int &i_millisecond) {

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


void CTimeString::getTimeValueFromString(string string_,int &i_minute,int &i_second,int &i_millisecond) {

	vector<int> find_vec = find_all(string_, "_");
	string s_minute, s_second, s_millisecond;
	s_minute = string_.substr(find_vec[0] + 3, 2);
	s_second = string_.substr(find_vec[1] + 1, 2);
	s_millisecond = string_.substr(find_vec[2] + 1, 3);

	i_minute = stoi(s_minute);
	i_second = stoi(s_second);
	i_millisecond = stoi(s_millisecond);
}


void CTimeString::setTime() {

	SYSTEMTIME st;
	GetSystemTime(&st);

	m_year = st.wYear;
	m_month = st.wMonth;
	m_day = st.wDay;
	m_hour = st.wHour+9;
	m_minute = st.wMinute;
	m_second = st.wSecond;
	m_milliseconds = st.wMilliseconds;

	if (m_hour >= 24) {
		m_hour -= 24;
		m_day++;
	}

}

//https://www.sejuku.net/blog/49318
std::vector<int> CTimeString::find_all(const std::string str, const std::string subStr) {
	std::vector<int> result;

	int subStrSize = subStr.size();
	int pos = str.find(subStr);

	while (pos != std::string::npos) {
		result.push_back(pos);
		pos = str.find(subStr, pos + subStrSize);
	}

	return result;
}

bool CTimeString::getFileNames(std::string folderPath, std::vector<std::string> &file_names)
{	

	//https://qiita.com/tes2840/items/8d295b1caaf10eaf33ad
	//#include <windows.h>
	//#include <vector>
	//#include <string>
	//#include <iostream>
	HANDLE hFind;
	WIN32_FIND_DATA win32fd;
	std::string search_name = folderPath + "\\*";

	hFind = FindFirstFile(search_name.c_str(), &win32fd);

	if (hFind == INVALID_HANDLE_VALUE)
	{
		cout << "directory not found" << endl;

		vector<int> find_vec = find_all(folderPath, "/");
		for (int i = 0; i < find_vec.size(); i++)
		{
			cout << "not found:" << folderPath << endl;

			string folderPath_each = folderPath.substr(0, find_vec[find_vec.size() - 1 - i]);
			std::string search_name_each = folderPath_each + "\\*";
			HANDLE hFind_each;
			WIN32_FIND_DATA win32fd_each;
			hFind_each = FindFirstFile(search_name_each.c_str(), &win32fd);
			if (hFind_each == INVALID_HANDLE_VALUE)
				cout << "not found:" << folderPath_each << endl;
			else
			{
				cout << "    found:" << folderPath_each << endl;
				break;
			}
		}
		throw std::runtime_error("file not found");
		return false;		
	}

	do
	{
		if (win32fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
			cout << win32fd.cFileName << "(directory)" << endl;
		}
		else file_names.push_back(win32fd.cFileName);

	} while (FindNextFile(hFind, &win32fd));

	FindClose(hFind);

	return true;
}

bool CTimeString::getFileNames_extension(std::string folderPath, std::vector<std::string> &file_names, string s_extension)
{
	//https://www.sejuku.net/blog/49318
	vector<string> filenames_;
	bool b_success = getFileNames(folderPath, filenames_);
	for (int i = 0; i < filenames_.size(); i++)
	{
		int i_find = filenames_[i].find(s_extension);
		if (i_find == std::string::npos) continue;
		file_names.push_back(filenames_[i]);
		cout << file_names.back() << endl;
	}
	return b_success;
}


bool CTimeString::getIsStringValueOrNot(string s_string)
{
	////http://program.station.ez-net.jp/special/handbook/cpp/string/all_of.asp
	if (std::all_of(s_string.cbegin(), s_string.cend(), isdigit)) return true;

	{
		vector<int> find_vec_1 = find_all(s_string, "-");
		if (find_vec_1.size() == 1)
			if (find_vec_1[0] == 0)	s_string = s_string.substr(1, s_string.size() - 1);
	}

	vector<int> find_vec = find_all(s_string, ".");

	if (find_vec.size() != 1) return false;

	string s_former = s_string.substr(0, find_vec[0] - 0);
	string s_later = s_string.substr(find_vec[0]+1, s_string.size() - (find_vec[0] + 1));

	if (!std::all_of(s_former.cbegin(), s_former.cend(), isdigit)) return false;
	if (!std::all_of(s_later.cbegin(), s_later.cend(), isdigit)) return false;

	return true;
}
