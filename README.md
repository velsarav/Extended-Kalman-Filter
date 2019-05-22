# Extended Kalman Filter 
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

**Extended Kalman Filter**

The goal of this project is to utilize a Kalman filter to estimate the state of a moving object of interest with noisy LIDAR and RADAR measurements in C++.

## Rubric Points

The [rubric](https://review.udacity.com/#!/rubrics/748/view) points were individually addressed in the [implementation](https://github.com/velsarav/project-extended-kalman-filter/tree/master/src)

Please refer to the base project [README](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project) to compile C++ code and run the simulator.

[//]: # (Image References)

[image1]: ./Docs/RMSE_result.png "RMSE value"
[image2]: ./Docs/Sensor_properties.png "Sensor properties"
[image3]: ./Docs/Estimation.png "Estimation"
[image4]: ./Docs/KF.png "KF"
[image5]: ./Docs/Prediction_1.png "Prediction"
[image6]: ./Docs/Update_1.png "Update"
[image7]: ./Docs/SensorFusion.png "Fusion"
[image8]: ./Docs/KF_Overview.png "KF Overview"
[image9]: ./Docs/NonGaussian.png "Non Gaussian"
[image10]: ./Docs/EKF_Linear.png "EKF linear"
[image11]: ./Docs/Linear_Approximation.png "Linear Approximation"
[image12]: ./Docs/KFVsEKF.png "KF VS EKF"
[image13]: ./Docs/EKFEquation.png "EKF Equation"
[image14]: ./Docs/EquationSummary.png "Equation Summary"
[image15]: ./Docs/FinalFlow.png "Sensor Fusion Flow"



## Sensor Fusion
Sensor such as LIDAR and RADAR have different properties which helps to provide different types of information about the tracked object position with differing accuracies especially in different weather condition.

![alt text][image2]

The techniques used to merge information from different sensor is called sensor fusion. 

![alt text][image7]

Next step we are going to detect a bicycle that travels around the autonomous vehicle using the simulated LIDAR and RADAR measurement.

Kalman filter will be used to track the bicycle's position and velocity using LIDAR and RADAR measurement.

Overview of the Kalman Filter Algorithm map

![alt text][image8]

## Kalman Filter
A Kalman filter is an optimal estimation algorithm used to estimate states of a system from indirect and uncertain measurements. Kalman filter models the state uncertainty using Gaussians and it is capable of making accurate estimates with just few data points.

KF starts with an initial state estimate then perform  the following cycle:
measurement update produced by sensor measurements followed by a state prediction from control actions.

![alt text][image3]

#### Kalman Filter Algorithm steps

![alt text][image4]

First measurement - Filter receives the initial measurement of the bicycle's position relative to the car. The measurement comes from RADAR or LIDAR sensor.

Initialisation - Initialize state and covariance matrices of the filter based on  the first measurement of bicycle's position.

Then the car will receive another sensor measurement after a time period Δt.

State prediction - Will predict where the bicycle will be after time Δt. One basic way to predict the bicycle location after Δt is to assume the bicycle's velocity is constant; thus the bicycle will have moved velocity * Δt.

![alt text][image5]

Equation (1) provides the prediction calculation.

The bicycle didn't maintain the exact same velocity. May be the bicycle changed the direction, accelerated or decelerated. So when we predict the position after Δt, our uncertainty increases. Equation (2) represents this increase in uncertainty. 


x --> mean state vector. 

P --> state covariance matrix, which contains the information about the uncertainty of the object's position and velocity. You can think of it as containing standard deviations.

Q --> process covariance matrix. It is a covariance matrix associated with the noise in states. 

F --> transition Matrix (deals with time steps and constant velocities)

The notation u ∼ N(0,Q) defines the process noise as a gaussian distribution with mean zero and covariance Q.

Measurement update - Compares the "predicted" location with the sensor measurement. The predicted location and measured location will provide a updated location. The Kalman filter will put more weight on either the predicted location or the measured location depending on the uncertainty of each value.

We are going to obtain the data from LIDAR measurements.

*LIDAR Measurements* allows us to get the current position but not its velocity. On the other hand it have more resolution. 

![alt text][image6]

Use the real measurement z to update the predicted state x' by a scaling factor called the Kalman Gain proportional to the error between the measurement and the predicted state.

Equation (3) get the error value y (also called the residual) based on z and H where

z --> Measurement vector. For a LIDAR sensor, the z vector contains the *position - x* and *position - y* measurements.

H --> matrix that projects your belief about the bicycle's current state into the measurement space of the sensor. For LIDAR, this is a fancy way of saying that we discard velocity information from the state variable since the LIDAR sensor only measures position: The state vector x contains information about [px​,py​,vx​,vy​] whereas the z vector will only contain [px,py]. Multiplying Hx allows us to compare x, our belief, with z, the sensor measurement.

Equation (4), (5), (6) and (7) provides the Kalman gain.

R --> Covariance matrix of the measurement noise.
S --> Projection of the process uncertainty into the measurement space.

Equation (6) update the predict state and Equation (7) update the process uncertainty.

*RADAR measurements* goes further and allows us to get the velocity information as well. Extended Kalman filter helps to understand the measurement update part of RADAR.

## Extended Kalman Filter
KF assumes the motion and measurement models are *linear* and that the state space can be represented by a unimodal *Gaussian distribution*. Self-driving car will execute *non linear motion*. Non linear actions will result in *non-Gaussian posterior distributions* that cannot be properly modelled by a closed form equation. 

![alt text][image9]

EKF approximates motion and measurements to linear functions locally (i.e. by using the first two terms of a Taylor series) to compute a best Gaussian approximation of the posterior covariance. 

![alt text][image10]

This trick allows EKF to be applied efficiently to a non-linear problems.

![alt text][image11]

KF Vs EKF Equation

![alt text][image12]

The main differences are:

* The F matrix will be replaced by Fj​ when calculating P′.
* The H matrix in the Kalman filter will be replaced by the Jacobian matrix Hj​ when calculating S, K, and P.
* To calculate x′, the prediction update function f, is used instead of the F matrix.
* To calculate y, the h function is used instead of the H matrix.

For this project, however, we do not need to use the f function or Fj. If we had been using a non-linear model in the prediction step, we would need to replace the F matrix with its Jacobian Fj
​
However, we are using a linear model for the prediction step. So, for the prediction step, we can still use the regular Kalman filter equations and the F matrix rather than the extended Kalman filter equations.

The measurement update for LIDAR will also use the regular Kalman filter equations, since LIDAR uses linear equations. Only the measurement update for the RADAR sensor will use the extended Kalman filter equations.

![alt text][image13]

To summarize:

![alt text][image14]

## Programming Kalman Filter

Sensor fusion general flow for this project

![alt text][image15]

Following were the three main steps for programming a Kalman Filter

* **initializing** Kalman filter variables
* **predicting** where our object is going to be after a time step Δt
* **updating** where our object is based on sensor measurements

Then the prediction and update steps repeat themselves in a loop.

To measure how well our Kalman filter performs, we will then calculate **root mean squared error (RMSE)** comparing the Kalman filter results with the provided ground truth.

These three steps (initialize, predict, update) plus calculating RMSE encapsulate the entire extended Kalman filter project.

Files in the **src** folder:

* main.cpp - communicates with the Udacity Term 2 Simulator receiving data measurements, calls a function to run the Kalman filter, calls a function to calculate RMSE

* FusionEKF.cpp - initializes the filter, calls the predict function, calls the update function. It has a variable called ekf_ which is an instance of a KalmanFilter class. 

      float ro     = measurement_pack.raw_measurements_(0);
      float phi    = measurement_pack.raw_measurements_(1);
      float ro_dot = measurement_pack.raw_measurements_(2);
      ekf_.x_(0) = ro     * cos(phi);
      ekf_.x_(1) = ro     * sin(phi);      
      ekf_.x_(2) = ro_dot * cos(phi);
      ekf_.x_(3) = ro_dot * sin(phi);

* kalman_filter.cpp- defines the predict function, the update function for LIDAR, and the update function for RADAR

* tools.cpp- function to calculate RMSE and the Jacobian matrix


## Accuracy

The project rubric on accuracy :

**Your algorithm will be run against Dataset 1 in the Udacity simulator which is the same as "data/obj_pose-laser-radar-synthetic-input.txt" in the repository. We'll collect the positions that your algorithm outputs and compare them to ground truth data. Your px, py, vx, and vy RMSE should be less than or equal to the values [.11, .11, 0.52, 0.52].**

## C++ compilation
Able to install uWebSocketIO in [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) and run the [Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases)


Following is the accuracy test result.

![alt text][image1]

### Reference
* [EKF](https://medium.com/intro-to-artificial-intelligence/extended-kalman-filter-simplified-udacitys-self-driving-car-nanodegree-46d952fce7a3)

* [Kalman filter and Particle filter](https://medium.com/@fernandojaruchenunes/udacity-robotics-nd-project-6-where-am-i-8cd657063585)

* [Sensor Fusion](https://medium.com/@wilburdes/sensor-fusion-algorithms-for-autonomous-driving-part-1-the-kalman-filter-and-extended-kalman-a4eab8a833dd)

* [Extended Kalman Filter](https://towardsdatascience.com/extended-kalman-filter-43e52b16757d)

* [KF Big picture](https://medium.com/@tempflip/udacity-self-driving-cars-extended-kalman-filters-my-bits-99cbbaf65e3d)

* [Interactive tutorial](https://home.wlu.edu/~levys/kalman_tutorial/)
