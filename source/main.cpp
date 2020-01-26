#include"Velodyne_Interface.h"

int main() {

	//string ipaddress("192.168.1.79");
	string ipaddress("192.168.1.70");
	string port("2368");
	//string port("8308");
	//string ipaddress("192.168.1.201");		//kougei dai

	CVelodyneInterface velodyne;
	velodyne.all(ipaddress, port);

	return 0;
}