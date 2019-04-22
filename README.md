# Extended Kalman Filter 
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

**Extended Kalman Filter**

The goal of this project is to utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements in C++.

## Rubric Points

The [rubric](https://review.udacity.com/#!/rubrics/748/view) points were individually addressed in the [implementation](https://github.com/velsarav/project-extended-kalman-filter/tree/master/src)

Please refer to the base project [README](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project) to compile C++ code and run the simulator.

[//]: # (Image References)

[image1]: ./Docs/RMSE_result.png "RMSE value"
[image2]: ./Docs/Sensor_properties.png "Sensor properties"

### Sensor Fusion
Different sensor have different properties which helps to provide different types of information about the tracked object position with differing accuracies especially in different weather condition.

![alt text][image2]

### Kalman Filter
The Kalman filter estimates the value of variable by updating its estimate as measurement data is collected filtering out the noise. Kalman filter models the state uncertainty using Gaussians and it is capable of making accurate estimates with just few data points.

KF starts with an initial state estimate then perform  the following cycle:
measurement update produced by sensor measurements followed by a state prediction from control actions.

![alt text][image3]

### Extended Kalman Filter
KF assumes the motion and measurement models are linear and that the state space can be represented by a unimodal Gaussian distribution. Self-driving car will execute non linear motion. Non linear actions will result in non-Gaussian posterior distributions that cannot be properly modeled by a closed form equation. 

EKF approximates motion and measurements to linear functions locally (i.e. by using the first two terms of a Taylor series) to compute a best Gaussian approximation of the posterior covariance. This trick allows EKF to be applied efficiently to a non-linear problems.

![alt text][image4]

### Accuracy

![alt text][image1]

### Reference
[EKF](https://medium.com/intro-to-artificial-intelligence/extended-kalman-filter-simplified-udacitys-self-driving-car-nanodegree-46d952fce7a3)