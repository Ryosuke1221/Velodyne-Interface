#include"Velodyne_Interface.h"

int main() {

	string ipaddress("192.168.1.79");
	string port("2368");
	//string ipaddress("192.168.1.201");		//kougei dai

	CVelodyneInterface velodyne;
	velodyne.all(ipaddress, port);

	return 0;
}