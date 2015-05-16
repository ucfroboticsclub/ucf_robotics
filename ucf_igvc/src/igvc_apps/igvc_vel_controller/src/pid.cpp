/**

Copyright (c) 2015, Robotics Club @ UCF
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.

@author Thomas Watters (thomaswatters@knights.ucf.edu)

*/


/*
 * Implements a PID Controller. Control loop is threaded.
 */
#include "pid.h"
#include "boost/timer.hpp"
//#include <iostream>

pid::pid(float kp, float ki, float kd)
        : ki_(ki), kp_(kp), kd_(kd),
          setpoint_(0.0), actual_(0.0), output_(0.0), running_(false)
{

}

pid::~pid()
{
    running_ = false;
    thr_->join();
    delete thr_;
}


/**
 *  Basic implementation of PID control
 */
void pid::run()
{
    running_ = true;

    double previous_error = 0;
    double integral = 0;

    boost::timer clock;
    boost::posix_time::milliseconds time(5);
    while (running_)
    {
        double dt = clock.elapsed();
        clock.restart();

        mutex_.lock();
        double error = setpoint_ - actual_;
        integral = integral + error * dt;
        double derivative = (error - previous_error) / dt;
        output_ = kp_ * error + ki_ * integral + kd_ * derivative;
        previous_error = error;
       // std::cout << "S: " << setpoint_ << " A: " << actual_ << " O: " << output_ << " E: " << error << std::endl;

        mutex_.unlock();

        boost::this_thread::sleep(time);

    }

}

void pid::start()
{
    thr_ = new boost::thread(&pid::run, this);
}

void pid::join()
{
    running_ = false;
    thr_->join();
}

void pid::SetDesired(double val)
{
    mutex_.lock();
    setpoint_ = val;
    mutex_.unlock();

}

void pid::SetActual(double val)
{
    mutex_.lock();
    actual_ = val;
    mutex_.unlock();
}

double pid::GetOutput()
{
    double ret;
    mutex_.lock();
    ret = output_;
    mutex_.unlock();

    return ret;
}
