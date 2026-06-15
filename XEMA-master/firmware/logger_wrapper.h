
#ifndef LOGGER_WRAPPER_H
#define LOGGER_WRAPPER_H
#pragma once

#include"easylogging++.h"

//定义额外日志文件的保存
#define EXT_ERROR_LOGGER "error_log"
//LOG_ERROR将保存双份，既在标准输出，又在EXT_ERROR_LOGGER中

#define LOG_ERROR DualErrorLogger()
// 自定义 DualLogger 流式包装
class DualErrorLogger {
public:
	template<typename T>
	DualErrorLogger& operator<<(const T& value) {
		buffer_ << value<< " ";
		return *this;
	}

	~DualErrorLogger() {
		std::string msg = buffer_.str();
		LOG(ERROR)<<msg;
		el::Loggers::getLogger(EXT_ERROR_LOGGER)->error(msg);
	}
private:
	std::ostringstream buffer_;
};

#endif
