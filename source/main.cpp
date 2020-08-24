#include"Velodyne_Interface.h"

int main() 
{

	string ipaddress("192.168.1.77");
	string port("2368");

	CVelodyneInterface velodyne;
	velodyne.all(ipaddress, port);

	return 0;
}