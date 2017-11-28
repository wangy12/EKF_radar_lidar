# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project, a kalman filter and a extended Kalman filter are implemented to estimate the state of a moving object of interest with noisy lidar and radar measurements. The RMSE values are evaluated. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

[//]: # (Image References)

[image1]: ./pics/traj_p1.png 



[image6]: ./pics/final_result.png 
[image7]: ./pics/final_result2.png 
[image8]: ./pics/final_result_dataset2.png 

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF



Note that the programs that need to be written to accomplish the project are src/FusionEKF.cpp, src/FusionEKF.h, kalman_filter.cpp, kalman_filter.h, tools.cpp, and tools.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

While I test my code, the following results show up.

![alt text][image1]



This problem happens at the 274th radar data, where the bearing is 3.190031e+00. Since the 'atan2()' returns values between $-\pi$ and $\pi$, but the given measurement bearing is greater than $\pi$. I add the following to the code to yield the bearing within a desired range.

```
if (abs(y(1)) > M_PI){
	y(1) = y(1) - 2 * M_PI;
}
```

Finally, the RMSE is: 

![alt text][image6]

![alt text][image7]

![alt text][image8]
