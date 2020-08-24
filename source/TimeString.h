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
	static void movefile(string path_before, string path_after);

	static vector<string> inputSomeString();
	static vector<string> inputSomeString_fromCSV(string s_filename);
	static void showParameter(vector<float> parameter_vec, vector<string> name_vec,int i_frame_show = -1);
	static void changeParameter(vector<float> &parameter_vec, vector<string> name_vec);
	static void changeParameter(vector<float> &parameter_vec, vector<string> name_vec, string filename_, 
		int row_small, int row_big, int col_);
	static void changeParameter_2dimension(vector<vector<float>> &parameter_vec_vec, vector<string> name_vec, vector<float> parameter_vec_init);
	static void changeParameter_2dimension(vector<vector<float>> &parameter_vec_vec, vector<string> name_vec, vector<float> parameter_vec_init,
		string filename_, int row_small, int col_small, int row_big, int col_big);
	static vector<vector<float>> inputParameters_2dimension(string filename_, int row_small, int col_small);

	template<typename T>
	static vector<vector<T>> calcVectorPairPattern(vector<vector<T>> vectorPair_vecvec)
	{
		vector<vector<T>> pattern_vecvec;

		//make pattern_vec_vec
		int num_pattern;
		num_pattern = 1;
		for (int j = 0; j < vectorPair_vecvec.size(); j++)
			num_pattern *= vectorPair_vecvec[j].size();
		pattern_vecvec.resize(num_pattern);
		for (int j = 0; j < num_pattern; j++)
			pattern_vecvec[j].resize(vectorPair_vecvec.size());

		//insert to pattern_vec_vec
		for (int j = 0; j < num_pattern; j++)
		{
			int i_devide = num_pattern;
			int i_residual = j;
			for (int i = 0; i < vectorPair_vecvec.size(); i++)
			{
				int idx;
				i_devide /= vectorPair_vecvec[i].size();
				idx = i_residual / i_devide;
				i_residual %= i_devide;
				pattern_vecvec[j][i] = vectorPair_vecvec[i][idx];
			}
		}

		//removeSameParameter(vectorPair_vecvec);
		removeSameValue_vecvec_VectorPairPattern(vectorPair_vecvec);

		cout << "show pattern" << endl;
		for (int j = 0; j < pattern_vecvec.size(); j++)
		{
			string s_j = to_string(j);
			if (s_j.size() < 4) s_j = " " + s_j;
			if (s_j.size() < 4) s_j = " " + s_j;
			if (s_j.size() < 4) s_j = " " + s_j;
			cout << s_j << ":";
			for (int i = 0; i < pattern_vecvec[j].size(); i++)
			{
				string s_value;
				s_value = to_string(pattern_vecvec[j][i]);
				if (s_value.size() < 4) s_value = " " + s_value;
				if (s_value.size() < 4) s_value = " " + s_value;
				if (s_value.size() < 4) s_value = " " + s_value;
				cout << "  " << s_value;

			}
			cout << endl;
		}
		cout << endl;

		return pattern_vecvec;
	}

	template<typename T>
	static void removeSameValue_vecvec_VectorPairPattern(vector<vector<T>> &vectorPair_vecvec)
	{
		for (int j = 0; j < vectorPair_vecvec.size(); j++)
			removeSameValue_vec_VectorPairPattern(vectorPair_vecvec[j]);
	}

	template<typename T>
	static void removeSameValue_vec_VectorPairPattern(vector<T> &vectorPair_vec)
	{
		vector<T> vectorPair_vec_new;
		//sort
		sort(vectorPair_vec.begin(), vectorPair_vec.end());
		//remove same value
		T value_last;
		for (int i = 0; i < vectorPair_vec.size(); i++)
		{
			if (i == 0)
				vectorPair_vec_new.push_back(vectorPair_vec[i]);
			else
				if (value_last != vectorPair_vec[i])
					vectorPair_vec_new.push_back(vectorPair_vec[i]);
			value_last = vectorPair_vec[i];
		}
		vectorPair_vec = vectorPair_vec_new;
	}

	template<typename T>
	static void sortVector(vector<T> &value_vec, bool b_ascending = true)
	{
		for (int i = 0; i < value_vec.size(); i++)
		{
			for (int j = value_vec.size() - 1; j > i; j--)
			{
				bool b_swap = false;
				if (b_ascending && value_vec[j] < value_vec[j - 1])
					b_swap = true;
				else if (!b_ascending && value_vec[j] > value_vec[j - 1])
					b_swap = true;
				if (b_swap) swap(value_vec[j], value_vec[j - 1]);
			}
		}
	}

	template<typename T>
	static void sortVector2d(vector<vector<T>> &value_vecvec, int index_arg, bool b_ascending = true)
	{
		vector<pair<T, int>> frame_pair_vec;
		for (int j = 0; j < value_vecvec.size(); j++)
			frame_pair_vec.push_back(make_pair(value_vecvec[j][index_arg], j));
		for (int i = 0; i < frame_pair_vec.size(); i++)
		{
			for (int j = frame_pair_vec.size() - 1; j > i; j--)
			{
				bool b_swap = false;
				if (b_ascending && frame_pair_vec[j].first < frame_pair_vec[j - 1].first)
					b_swap = true;
				else if(!b_ascending && frame_pair_vec[j].first > frame_pair_vec[j - 1].first)
					b_swap = true;
				if(b_swap) swap(frame_pair_vec[j], frame_pair_vec[j - 1]);
			}
		}
		vector<vector<T>> value_input_vecvec_new;
		for (int j = 0; j < frame_pair_vec.size(); j++)
		{
			value_input_vecvec_new.push_back(value_vecvec[frame_pair_vec[j].second]);
		}
		value_vecvec.clear();
		value_vecvec = value_input_vecvec_new;
	}

	template<typename T>
	static void sortVector2d_2dimension(vector<vector<T>> &value_vecvec, int index_first, int index_second, bool b_ascending = true)
	{
		vector<vector<T>> value_vecvec_new;
		sortVector2d(value_vecvec, index_first, b_ascending);
		vector<vector<int>> index_sameFirstValue_vecvec;
		{
			vector<int> index_sameFirstValue_vec;
			T value_pickup;
			for (int j = 0; j < value_vecvec.size(); j++)
			{
				if (j == 0) value_pickup = value_vecvec[j][index_first];
				else if (value_pickup != value_vecvec[j][index_first])
				{
					index_sameFirstValue_vecvec.push_back(index_sameFirstValue_vec);
					index_sameFirstValue_vec.clear();
					value_pickup = value_vecvec[j][index_first];

				}
				index_sameFirstValue_vec.push_back(j);
				if (j == value_vecvec.size() - 1)
					index_sameFirstValue_vecvec.push_back(index_sameFirstValue_vec);
			}
		}
		for (int j = 0; j < index_sameFirstValue_vecvec.size(); j++)
		{
			vector<vector<T>> value_vecvec_1firstValue;
			for (int i = 0; i < index_sameFirstValue_vecvec[j].size(); i++)
				value_vecvec_1firstValue.push_back(value_vecvec[index_sameFirstValue_vecvec[j][i]]);
			sortVector2d(value_vecvec_1firstValue, index_second, b_ascending);
			value_vecvec_new.insert(value_vecvec_new.end(), value_vecvec_1firstValue.begin(), value_vecvec_1firstValue.end());
		}
		value_vecvec.clear();
		value_vecvec = value_vecvec_new;
	}

	template<typename T>
	static T getMedian(vector<T> value_vec)
	{
		sortVector(value_vec);

		int size = value_vec.size();

		if (size % 2 == 1)
			return value_vec[(size - 1) / 2];
		else
			return (value_vec[(size / 2) - 1] + value_vec[size / 2]) / 2.;
	}

	template<typename T>
	static vector<T> getMedian_Quartile(vector<T> value_vec)
	{
		//https://atarimae.biz/archives/19162
		sortVector(value_vec);
		int size = value_vec.size();
		T median_ = getMedian(value_vec);
		T first_quartile;
		T third_quartile;
		if (size % 2 == 1)
		{
			vector<T> temp_vec_first;
			temp_vec_first.insert(temp_vec_first.begin(), value_vec.begin(), value_vec.begin() + (size - 1) / 2);
			first_quartile = getMedian(temp_vec_first);
			vector<T> temp_vec_third;
			temp_vec_third.insert(temp_vec_third.begin(), value_vec.begin() + (size - 1) / 2 + 1, value_vec.end());
			third_quartile = getMedian(temp_vec_third);
		}
		else
		{
			vector<T> temp_vec_first;
			temp_vec_first.insert(temp_vec_first.begin(), value_vec.begin(), value_vec.begin() + (size / 2));
			first_quartile = getMedian(temp_vec_first);
			vector<T> temp_vec_third;
			temp_vec_third.insert(temp_vec_third.begin(), value_vec.begin() + size / 2, value_vec.end());
			third_quartile = getMedian(temp_vec_third);
		}
		vector<T> output_vec;
		output_vec.push_back(first_quartile);
		output_vec.push_back(median_);
		output_vec.push_back(third_quartile);
		return output_vec;
	}
	
	static vector<vector<int>> getIntCluster_SomeToSome(vector<vector<int>> value_vecvec, bool b_recursive = false);

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
		int rows = 0;
		int cols = 0;
		rows = T_vecvec.size();

		for (int j = 0; j < T_vecvec.size(); j++)
			if (cols < T_vecvec[j].size()) cols = T_vecvec[j].size();

		//fill blank ingredient
		for (int j = 0; j < T_vecvec.size(); j++)
		{
			int error_cols = cols - T_vecvec[j].size();
			for (int i = 0; i < error_cols; i++)
			{
				T temp;
				T_vecvec[j].push_back(temp);
			}
		}

		swap(rows, cols);

		for (int j = 0; j < rows; j++)
		{
			vector<T> T_vec_output;
			for (int i = 0; i < cols; i++)
				T_vec_output.push_back(T_vecvec[i][j]);
			T_vecvec_output.push_back(T_vec_output);
		}
		return T_vecvec_output;
	}

	static vector<vector<string>> getMatrixData_fromFormatOfFPFH(vector<vector<string>> s_input_vecvec,
		string s_start, int i_pos_start_fromS, string s_finish, int i_pos_finish_fromS);

	template<typename T>
	static vector<int> getHostogram(const vector<T> value_vec, int num_bin, bool b_cout = false)
	{
		T value_max = -std::numeric_limits<T>::max();
		T value_min = std::numeric_limits<T>::max();

		for (int j = 0; j < value_vec.size(); j++)
		{
			if (value_max < value_vec[j]) value_max = value_vec[j];
			if (value_min > value_vec[j]) value_min = value_vec[j];
		}

		cout << "value_max:" << value_max << endl;
		cout << "value_min:" << value_min << endl;

		T range_ = (value_max - value_min) / (T)num_bin;

		vector<int> hist_vec;
		hist_vec.resize(num_bin);
		fill(hist_vec.begin(), hist_vec.end(), 0);

		for (int j = 0; j < value_vec.size(); j++)
		{
			int i_bin = (int)((value_vec[j] - value_min) / range_);
			if (value_vec[j] == value_max) i_bin--;
			if (i_bin < 0 || num_bin - 1 < i_bin)
			{
				cout << "invalid: i_bin = " << i_bin << endl;
				cout << "j:" << j << " value_vec[j]:" << value_vec[j] << endl;
			}
			hist_vec[i_bin]++;
		}

		if (b_cout)
		{
			for (int j = 0; j < hist_vec.size(); j++)
			{
				string s_index = to_string(j);
				if (s_index.size() < 2) s_index = " " + s_index;

				T value_ = value_min + range_ * j;
				string s_value = to_string(value_);
				for (int i = 0; i < 9; i++)
					if (s_value.size() < 9) s_value = " " + s_value;

				string s_num = to_string(hist_vec[j]);
				for (int i = 0; i < 6; i++)
					if (s_num.size() < 6) s_num = " " + s_num;

				cout << "[" << s_index << "] value:" << s_value << " num:" << s_num << endl;

			}
		}

		return hist_vec;
	}

private:
	static bool getDirectoryExistance(string foder_Path);
	static bool getDirectoryExistance_detail(string foder_Path, bool b_first);


};
