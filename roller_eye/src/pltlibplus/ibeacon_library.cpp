#include "ibeacon_library.h"
#include "kalman.h"
#include <cmath>
#include <string>
#include <iostream>
#include <vector>

using namespace std;
string libdir = "/usr/local/bin/"; //edit directory where ibeacon_library can be found
Kalman myFilter(0.065,1.4,-100,0); //kalman filter parameters
float range = 0.0;

int ibeacon_start() {
	//turn on bluetooth
	// ROS_INFO("Enabling beacon...");
	//system("/home/linaro/start_blue.sh >/dev/null 2>&1 &");
        //sleep(20);
	system("sudo hciconfig hci0 reset");
	system("sudo hcitool lescan --duplicates > /dev/null &");

	sleep(1);
	myFilter.setParameters(0.065,1.4,-50); //kalman filter parameters
	// ROS_INFO("Scanning...");

	return 0;
}

int ibeacon_stop() {
	// ROS_INFO("Stopping...");
	system("sudo pkill --signal SIGINT hcitool");
	system("sudo hciconfig hci0 down");
	// ROS_INFO("Scan stopped, hci0 down");
	// system("sudo killall hcidump");
	return 0;
}

string exec(string cmd) {
	char buffer[128];
	string result = "";
	FILE* pipe = popen(cmd.c_str(), "r");
	if (!pipe) throw runtime_error("popen() failed!");
	try {
	while (fgets(buffer, sizeof buffer, pipe) != NULL) {
	    result += buffer;
	}
	} catch (...) {
	pclose(pipe);
	throw;
	}
	pclose(pipe);
	return result;
}

float ibeacon_range(string uuid, string major, string minor, float n){
        //gets transmit power and rssi from ibeacon_scan_v2
	//kalman filter implemented in this fnx

        string ibeacon_line = exec(libdir+"ibeacon_scan_v2 "+uuid+" "+major+" "+minor);

	istringstream iss(ibeacon_line);
    	vector<string> result;
    	for(string s;iss>>s;) result.push_back(s);

	static float rangedata, rssi, txPower, range, filteredrssi;
	txPower = stof(result[1].c_str());
	rssi = stof(result[2].c_str());
	if ((txPower ==0) && (rssi==1000)) return -1; //when target beacon cant be detected
	if ((txPower ==0) && (rssi==2000)) return -2; //when ibeacon_start was not run properly

	filteredrssi = myFilter.getFilteredValue(rssi);

	//n = 10; // environmental factor. ranges from around 2-4
	range = pow(10, ((txPower - filteredrssi) / (10 * n)));
        return range;
}
