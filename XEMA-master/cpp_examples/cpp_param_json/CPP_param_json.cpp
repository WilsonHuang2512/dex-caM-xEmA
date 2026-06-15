
#include <iostream>
#include <string.h>
#include "xcamera.h"

using namespace XEMA;

int main()
{
	/*****************************************************************************************************/
	int ret_code = 0;

	XCamera* p_camera = (XCamera*)createXCamera();
	ret_code = p_camera->connect("192.168.3.70");

	//get param json
	char get_status_json[20480];
	char get_config_json[20480];
	p_camera->getParamJson(get_config_json, get_status_json); 
	p_camera->saveJson(get_config_json, "../get_config.json");
	p_camera->saveJson(get_status_json, "../get_status.json");

	//set patam json
	char status_json[20480];
	char config_json[20480];
	int num = 0;
	p_camera->readJson(config_json, "../get_config.json");
	p_camera->setParamJson(config_json, status_json,num);
	p_camera->saveJson(status_json, "../set_status.json");


	p_camera->disconnect("192.168.3.70");

	destroyXCamera(p_camera);

}


