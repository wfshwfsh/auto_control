#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

// 定义 PID 控制器结构
typedef struct {
    float kp; // 比例系数
    float ki; // 积分系数
    float kd; // 微分系数

    float previous_error; // 上一次误差
    float integral; // 积分累积
} PIDController;

// 初始化 PID 控制器
void pid_init(PIDController *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->previous_error = 0.0;
    pid->integral = 0.0;
}

// PID 控制算法
float pid_calculate(PIDController *pid, float setpoint, float measured_value, float dt) {
    // 计算误差
    float error = setpoint - measured_value;

    // 积分部分
    pid->integral += error * dt;

    // 微分部分
    float derivative = (error - pid->previous_error) / dt;

    // PID 输出
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    
    //printf(" error,integral,derivative,output = %f %f %f %f\n" 
    //            , error, pid->integral, derivative, output);
    
    // 更新前一个误差
    pid->previous_error = error;

    return output;
}

// 模拟系统
float simulate_system(float measured_value, float setpoint, float input) {
    // 简单的系统模型，模拟过程可以修改
    // 输入可以是速度、位置等控制量，这里仅作为演示
    return (measured_value + (input) * 0.01);
}

int main(int argc, char *argv[]) {
    PIDController pid;
    float setpoint = 100.0; // 期望值，比如目标温度、目标位置
    float measured_value = 0.0; // 实际测量值
    float control_output = 0.0;
    float dt = 0.1; // 时间步长


    // 初始化 PID 控制器 (kp, ki, kd)
    //pid_init(&pid, 3.0, 1.3, 0.5);
    pid_init(&pid, atof(argv[1]), atof(argv[2]), atof(argv[3]));

    // 模拟时间过程
    for (int i = 0; i < 10000; ++i) {
        // 计算 PID 输出
        control_output = pid_calculate(&pid, setpoint, measured_value, dt);

        // 模拟系统的响应，更新测量值
        measured_value = simulate_system(measured_value, setpoint, control_output);

        // 输出当前时刻的控制信息
        printf("Time: %.1f, Setpoint: %.1f, Measured: %.2f, Control Output: %.2f\n", i * dt, setpoint, measured_value, control_output);

        // 模拟延迟
        usleep(10000); // 10ms
    }

    return 0;
}
