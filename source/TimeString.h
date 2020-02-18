#pragma once
#include<iostream>
#include<windows.h>
#include<string>
#include<sstream>
#include<vector>
#include<typeinfo>
#include<algorithm>
#include<fstream>

using namespace std;

class CTimeString {

	int m_year, m_month, m_day, m_hour, m_minute, m_second, m_milliseconds;

	void setTime();

	void getTimeValueFromString(string string_, int &i_minute, int &i_second, int &i_millisecond);
	void getTimeValueFromString(string string_, int &i_hour, int &i_minute, int &i_second, int &i_millisecond);

public:

	//can use "std::to_string" for it ...but detail is unknown
	template < typename T > std::string to_string(const T& n)
	{
		std::ostringstream stm;
		stm << n;
		return stm.str();
	}


	string getTimeString();

	string getTimeElapsefrom2Strings(string s_former, string s_latter);	//output  second

	string getTElapsefrom2S(string s_former, string s_latter);

	std::vector<int> find_all(const std::string str, const std::string subStr);

	template<typename T>
	vector<vector<T>> getVecVecFromCSV(string filename_) {
		//double,float, int
		//https://qiita.com/hal1437/items/b6deb22a88c76eeaf90c
		//https://docs.oracle.com/cd/E19957-01/805-7887/6j7dsdhfl/index.html
		//https://pknight.hatenablog.com/entry/20090826/1251303641
		CTimeString time_;
		ifstream ifs_(filename_);
		string str_;
		int f_cnt = 0;
		vector<vector<T>> all_observation_vec_vec;
		if (ifs_.fail()) cout << "Error: file could not be read." << endl;
		else {
			while (getline(ifs_, str_)) {		//readed to string from file
				vector<T> one_observation_vec;
				vector<int> find_vec = time_.find_all(str_, ",");
				string s_value;

				s_value = str_.substr(0, find_vec[0]);
				if (!time_.getIsStringValueOrNot(s_value)) continue;

				if (typeid(T) == typeid(int))
					one_observation_vec.push_back(stoi(s_value));
				else if (typeid(T) == typeid(float))
					one_observation_vec.push_back(stof(s_value));
				else if (typeid(T) == typeid(double))
					one_observation_vec.push_back(stod(s_value));
				//else if (typeid(T) == typeid(string))
				//	one_observation_vec.push_back(s_value);

				int s_pos = 0;
				bool b_break = false;
				while (s_pos < find_vec.size() - 1)
				{
					s_value = str_.substr(find_vec[s_pos] + 1, find_vec[s_pos + 1] - (find_vec[s_pos] + 1));
					if (!time_.getIsStringValueOrNot(s_value))
					{
						b_break = true;
						break;
					}

					if (typeid(T) == typeid(int))
						one_observation_vec.push_back(stoi(s_value));
					else if (typeid(T) == typeid(float))
						one_observation_vec.push_back(stof(s_value));
					else if (typeid(T) == typeid(double))
						one_observation_vec.push_back(stod(s_value));
					//else if (typeid(T) == typeid(string))
					//	one_observation_vec.push_back(s_value);
					s_pos++;
				}
				if (b_break) continue;

				s_value = str_.substr(find_vec[s_pos] + 1, str_.size() - (find_vec[s_pos] + 1));
				if (!time_.getIsStringValueOrNot(s_value)) continue;

				if (typeid(T) == typeid(int))
					one_observation_vec.push_back(stoi(s_value));
				else if (typeid(T) == typeid(float))
					one_observation_vec.push_back(stof(s_value));
				else if (typeid(T) == typeid(double))
					one_observation_vec.push_back(stod(s_value));
				//else if (typeid(T) == typeid(string))
				//	one_observation_vec.push_back(s_value);

				all_observation_vec_vec.push_back(one_observation_vec);
			}
			ifs_.close();
		}
		return all_observation_vec_vec;
	}

	template<typename T>
	void getCSVFromVecVec(vector<vector<T>> saved_data_vec_vec, string filename_)
	{
		std::ofstream ofs_save;
		ofs_save.open(filename_, std::ios::out);
		for (int i = 0; i < saved_data_vec_vec.size(); i++)
		{
			for (int j = 0; j < saved_data_vec_vec[i].size(); j++)
			{
				ofs_save << saved_data_vec_vec[i][j];
				if (j < saved_data_vec_vec[i].size() - 1) ofs_save << ",";
			}
			ofs_save << endl;
		}
		ofs_save.close();

	}

	bool getFileNames(std::string folderPath, std::vector<std::string> &file_names);
	bool getFileNames_extension(std::string folderPath, std::vector<std::string> &file_names,string s_extension);

	bool getIsStringValueOrNot(string s_string);

};
