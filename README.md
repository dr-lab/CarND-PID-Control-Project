# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## PID Parameter Tuning

This project is to use PID controller to drive a car. There are P, I, D and error paramters.

    double kp = 0.225;  // proportional coefficient
    double ki = 0.00001; // integral coefficient
    double kd = 3.0; // differential coefficient

Overall the simulator gave data input by uWebSokcets, the key data from simulator is cte.

    CTE: Cross Track Error, basically can be understand as the distance of the car to the road track.

Note: There is also the steering angle from the simulator, but it is not been used.

For P.I.D each, the error is calculated as bellow (see PID.cpp # updateError(cte))

    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;

The calculate a total error for each round of incoming message from the simulator. (see PID.cpp # totalError())

    // Using: -tau_p * CTE - tau_d * diff_CTE - tau_i * int_CTE
    //return -Kp * p_error - Kd * d_error - Ki * i_error;
    return Kp * p_error + Kd * d_error + Ki * i_error;

Note: all these algorithms are from the class session.

Another parameter used in the implementation is speed "throttle". To make sure the car can reach some speed, and also not out of track, use bellow algorithm to tune the throttle

    double throttle = 0.3;
    double abs_cte = fabs(cte);
    if (abs_cte <= 0.01) {
        throttle = 0.5;
    } else if (abs_cte <= 0.4) {
        throttle = 0.4;
    } else {
        throttle = 0.3;
    }

This project is to find the appropriate values for three P.I.D. coefficients. And the final total error is calculated with them.

    P: If the proportional coefficient is too high, the car starts to oscillate and it cannot converge.
    I: Integral coefficient represents the sum of all errors. If it is too high, the car starts moving in circles.
    D: Differential coefficient needs to be little big to detect rapid oscillations in the CTE.

All the values of the coefficients are manually and based on many round of tryings, it is little empirical. Bellow values guarantees the car can finish the whole track successfully with an average speed of 40 miles/h.

    double kp = 0.225;  // proportional coefficient
    double ki = 0.00001; // integral coefficient
    double kd = 3.0; // differential coefficient

Here is one video show the PID drive the car on the whole track.

[![PID driving Video](https://img.youtube.com/vi/IN6Oo_bYUoY/0.jpg)](https://www.youtube.com/watch?v=IN6Oo_bYUoY)

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

In my implementation, I use XCode as IDE. After clone the project from github, at the root of the project folder, run following to generate a XCode project.

    cmake -G "Xcode" .
