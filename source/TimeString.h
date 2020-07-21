#pragma once
#include<iostream>
#include<windows.h>
#include<string>
#include<sstream>
#include<vector>
#include<typeinfo>

//20200519
#include <locale.h>
#include <tchar.h>
//
#include <filesystem>
#include <fstream>

using namespace std;
//namespace sys_ns = std::tr2::sys;	//occur error in JIROS
namespace sys_ns = std::experimental::filesystem;

class CTimeString {

	static void setTime(int& i_year, int& i_month, int& i_day, int& i_hour, int& i_minute, int& i_second, int& i_milliseconds);
	static void getTimeValueFromString(string string_, int &i_hour, int &i_minute, int &i_second, int &i_millisecond);

public:

	//can use "std::to_string" for it ...but detail is unknown
	template < typename T > static std::string to_string(const T& n)
	{
		std::ostringstream stm;
		stm << n;
		return stm.str();
	}

	static string getTimeString();

	static string getTimeElapsefrom2Strings(string s_former, string s_latter);	//output  second

	static std::vector<int> find_all(const std::string str, const std::string subStr);

	template<typename T>
	static void getCSVFromVecVec(vector<vector<T>> saved_data_vec_vec, string filename_)
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

	static bool getFileNames(std::string folderPath, std::vector<std::string> &file_names, bool b_cout = true, bool b_getDir = false, bool b_check = true);
	static bool getFileNames_extension(std::string folderPath, std::vector<std::string> &file_names, string s_extension);
	static bool getFileNames_folder(std::string folderPath, std::vector<std::string> &file_names);

	static int getTimeElapsefrom2Strings_millisec(string s_former, string s_latter);	//output  second

	static vector<vector<string>> getVecVecFromCSV_string(string filename_, string key_token = ",");
	static vector<vector<double>> getVecVecFromCSV(string filename_);
	
	static void copyfile(string filename_from, string filename_to);//full path
	static void deletefile(string filename_delete);//full path
	static void makenewfolder(string dir, string newfoldername);//relative path

	static vector<string> inputSomeString();
	static vector<string> inputSomeString_fromCSV(string s_filename);
	static void showParameter(vector<float> parameter_vec, vector<string> name_vec,int i_frame_show = -1);
	static void changeParameter(vector<float> &parameter_vec, vector<string> name_vec);
	static void changeParameter(vector<float> &parameter_vec, vector<string> name_vec, string filename_, 
		int row_small, int row_big, int col_);
	static void changeParameter_2dimension(vector<vector<float>> &parameter_vec_vec, vector<string> name_vec, vector<float> parameter_vec_init);
	static void changeParameter_2dimension(vector<vector<float>> &parameter_vec_vec, vector<string> name_vec, vector<float> parameter_vec_init,
		string filename_, int row_small, int col_small, int row_big, int col_big);
	static void calcParameterPattern(vector<vector<float>> &pattern_vec_vec,vector<vector<float>> parameter_vec_vec);
	static void removeSameParameter(vector<vector<float>> &parameter_vec_vec);
	static void sortStringVector2d(vector<vector<string>> &s_vecvec, int index_arg);
	static void sortStringVector2d_2ingredient(vector<vector<string>> &s_vecvec, int ing_large, int ing_small);

	template<typename T>
	static vector<vector<T>> getMatrixCSVFromVecVec(vector<vector<T>> saved_data_vec_vec)
	{
		vector<vector<string>> save_vec_vec;

		if (saved_data_vec_vec.size() != saved_data_vec_vec[0].size())
		{
			cout << "ERROR: rows and cols is different" << endl;
			return save_vec_vec;
		}

		for (int j = 0; j < saved_data_vec_vec.size() + 1; j++)
		{
			vector<string> save_vec;
			save_vec.resize(saved_data_vec_vec.size() + 1);
			save_vec_vec.push_back(save_vec);
		}

		//fill except value cell
		for (int i = 0; i < saved_data_vec_vec.size(); i++)
			save_vec_vec[0][i + 1] = to_string(i);
		for (int j = 0; j < saved_data_vec_vec.size(); j++)
			save_vec_vec[j + 1][0] = to_string(j);
		save_vec_vec[0][0] = "-";
		//fill value cell
		for (int j = 0; j < saved_data_vec_vec.size(); j++)
		{
			for (int i = 0; i < saved_data_vec_vec.size(); i++)
			{
				save_vec_vec[j + 1][i + 1] = to_string(saved_data_vec_vec[j][i]);
			}
		}
		return save_vec_vec;
	}

	template<typename T>
	static vector<vector<T>> getTranspositionOfVecVec(vector<vector<T>> T_vecvec)
	{
		vector<vector<T>> T_vecvec_output;
		int rows, cols;
		rows = T_vecvec[0].size();
		cols = T_vecvec.size();
		for (int j = 0; j < rows; j++)
		{
			vector<T> T_vec_output;
			for (int i = 0; i < cols; i++)
				T_vec_output.push_back(T_vecvec[i][j]);
			T_vecvec_output.push_back(T_vec_output);
		}
		return T_vecvec_output;
	}


private:
	static bool getDirectoryExistance(string foder_Path);
	static bool getDirectoryExistance_detail(string foder_Path, bool b_first);


};
