#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
	PIDController(double Kp, double Ki, double Kd, double setpoint)
		: Kp(Kp), Ki(Ki), Kd(Kd), setpoint(setpoint), previous_error(0.0), integral(0.0) {}

	PIDController() {};

	PIDController(const PIDController& pid_ctl)
	{
		Kp = pid_ctl.Kp;
		Ki = pid_ctl.Ki;
		Kd = pid_ctl.Kd;
		setpoint = pid_ctl.setpoint;
		previous_error = pid_ctl.previous_error;
		integral = pid_ctl.integral;
	}

	double update(double current_value) {
		// 计算当前误差
		double error = (setpoint - current_value) / 255.;

		// 计算积分部分
		integral += error;

		// 计算微分部分
		double derivative = error - previous_error;

		// 计算PID输出
		double output = (Kp * error) + (Ki * integral) + (Kd * derivative);

		// 更新前一次误差
		previous_error = error;

		return output;
	}

	bool clear_memory()
	{
		previous_error = 0;
		integral = 0;
		return true;
	}

public:
	double setpoint;  // 目标亮度

private:
	double Kp;  // 比例增益
	double Ki;  // 积分增益
	double Kd;  // 微分增益
	double previous_error;  // 前一次的误差
	double integral;  // 积分累积值
};

#endif // !PID_CONTROLLER_H
