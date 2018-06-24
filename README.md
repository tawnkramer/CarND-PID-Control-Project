# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

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

## Test Instructions
1. Run pid executable: `./pid`. 
2. Run [term2 simulator](https://github.com/udacity/self-driving-car-sim/releases) : ./term2_sim.x86_64
3. Select the fourth option Project 4 - PID Controller

## Results
The simulator models a car moving around the lake track. As the car moves, it calculates the cross track error, cte, or the distance from the center of the car to the center of the track. This value, along with velocity, and other telemetry is sent via websocket connection to the pid application. The pid application calculates an error and a control term for steering which is designed to return the car to the optimum position on the road.

![Results](pid-results.gif)

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)


## PID Controller Values

After many iterations with the simulator, the following values were chosen emperically:

|Parameter   | Symbol   | Value   |
|---|---|---|
| Proportional gain  | Kp  | 0.2  |
| Integral gain  | Ki  | 0.00001  |
| Differntial gain | Kd | 15.0  |

## Method For Tuning PID Values
1. Initially Kd, Ki are set to zero to isolate effect of Kp
2. Kp controls the strength of the correction towards target in proportion to the error. This value was chosen by trial and error to minimize deviatation from the center of the road without too much oscillation. Too high values cause over-reaction and high oscillation. Too low values result in driving off the turns. 
3. Kd is the differential term which serves to counter steer against the proportional term, which causes a more gentle approach when critically dampded. Too high value causes the Kp to fight and unable to converge. Too small a value and we approach and overshoot, causing oscillation.
4. Ki term serves to compensate for constant bias, such as wind or misalignment. This term only needs small value as it works with the cumulative error over time. Too high a value and it will over react and push vehicle off the track. Too small a value will allow a drifting to take car off course.

