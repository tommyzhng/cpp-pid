#ifndef PID_HPP
#define PID_HPP

#include <iostream>

class PID
{
public:
    /**
     * @brief PID controller constructor
     * @param dt Time step for the controller
     * @param max Maximum output value
     * @param min Minimum output value
     * @param Kp Proportional gain
     * @param Ki Integral gain
     * @param Kd Derivative gain
     * @param integ_clamp Integral clamp value
     * @param alpha_d Smoothing factor for the derivative term (larger alpha_d means less smoothing)
     * @param tau Time constant for low-pass filter on output (larger tau is more smoothing)
     */
    PID(float dt, float max, float min, float Kp, float Ki, float Kd, float integ_clamp, float alpha_d, float tau)
        : dt_(dt), max_(max), min_(min), Kp_(Kp), Ki_(Ki), Kd_(Kd), integ_clamp_(integ_clamp), derivative_(0.0), alpha_d_(alpha_d), alpha_lpf(dt / (dt + tau)), 
            integral_(0.0), prev_error_(0.0), old_error_filtered_(0.0), old_output_filtered_(0.0) {}

    float run(float setpoint, float currentValue) {
        error_ = setpoint - currentValue;

        if (dt_ > 0.0f) derivative_ = alpha_d_ * ((error_ - prev_error_) / dt_) + (1 - alpha_d_) * derivative_;
        output_ =  Kp_ * error_ + Ki_ * integral_ + Kd_ * derivative_;

        if (output_ > max_) output_ = max_; 
        else if (output_ < min_) output_ = min_;
        else integral_ += error_ * dt_;
        
        // integral clamp
        if (integral_ * Ki_ > integ_clamp_) integral_ = integ_clamp_ / Ki_;
        else if (integral_ * Ki_ < -integ_clamp_) integral_ = -integ_clamp_ / Ki_;

        output_ = alpha_lpf * output_ + (1 - alpha_lpf) * old_output_filtered_; // increasing tau makes output slower

        // Update old values for next iteration
        prev_error_ = error_;
        old_output_filtered_ = output_;

        return output_;
    }

    void reset() {
        integral_ = 0.0;
        prev_error_ = 0.0;
    }
    float integral_;

private:
    double dt_;
    float max_, min_;
    float Kp_, Ki_, Kd_, integ_clamp_;
    float error_, derivative_, alpha_d_, alpha_lpf;
    float prev_error_, old_error_filtered_, old_output_filtered_;
    float ef_, output_;

};

#endif