// param_json.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <fstream> 
#include <string>
#include "open_cam3d_sdk.h"
#include <iomanip>

int main()
{
	   
	int ret_code = DfConnect("192.168.0.108");

	//get param json
	char get_status_json[20480];
	char get_config_json[20480];
	DfGetParamJson(get_config_json, get_status_json); 
	DfSaveJson(get_config_json, "../get_config.json");
	DfSaveJson(get_status_json, "../get_status.json");

	//set patam json
	char status_json[20480];
	char config_json[20480];
	int num = 0;
	DfReadJson(config_json, "../get_config.json");
	DfSetParamJson(config_json, status_json,num);
	DfSaveJson(status_json, "../set_status.json");


	DfDisconnect("192.168.88.106");

	return 0;
}

 