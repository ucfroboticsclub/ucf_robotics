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

#ifndef PROJECT_PID_H
#define PROJECT_PID_H

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>


class pid {

private:
    float ki_, kp_, kd_;
    double setpoint_, actual_, output_;

    bool running_;

    //thread
    boost::thread *thr_;
    boost::mutex mutex_;


public:

    //constructor
    pid(float ki, float kp, float kd);

    ~pid();

    void SetDesired(double val);

    void SetActual(double val);

    //getters
    double GetOutput();

    void start();

    void run();

    void join();

};

#endif //PROJECT_PID_H
