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
[image3]: ./Docs/Estimation.png "Estimation"
[image4]: ./Docs/Prediction_1.png "Prediction"
[image5]: ./Docs/Update.png "Update"



### Sensor Fusion
Sensor such as Lidar and Radar have different properties which helps to provide different types of information about the tracked object position with differing accuracies especially in different weather condition.

![alt text][image2]

The techniques used to merge information from different sensor is called sensor fusion. 

Next step we are going to detect a bicycle that travels around the autonomous vehicle using the simulated lidar and radar measurement.

Kalman filter will be used to track the bicycle's position and velocity using lidar and radar measurement.

### Kalman Filter
The Kalman filter estimates the value of variable by updating its estimate as measurement data is collected filtering out the noise. Kalman filter models the state uncertainty using Gaussians and it is capable of making accurate estimates with just few data points.

KF starts with an initial state estimate then perform  the following cycle:
measurement update produced by sensor measurements followed by a state prediction from control actions.

![alt text][image3]

#### Kalman Filter Algorithm steps
First measurement - Filter receives the initial measurement of the bicycle's position relative to the car. The measurement comes from radar or lidar sensor.

Initialisation - Initialize state and covariance matrices of the filter based on  the first measurement of bicycle's position.

Then the car will receive another sensor measurement after a time period Δt.

State prediction - Will predict where the bicycle will be after time Δt. One basic way to predict the bicycle location after Δt is to assume the bicycle's velocity is constant; thus the bicycle will have moved velocity * Δt.

![alt text][image4]

Equation (1) provides the prediction calculation.

The bicycle didn't maintain the exact same velocity. May be the bicycle changed the direction, accelerated or decelerated. So when we predict the position after Δt, our uncertainty increases. Equation (2) represents this increase in uncertainty. 


x --> mean state vector. 

P --> state covariance matrix, which contains the information about the uncertainty of the object's position and velocity. You can think of it as containing standard deviations.

Q --> process covariance matrix. It is a covariance matrix associated with the noise in states. 

F --> transition Matrix (deals with time steps and constant velocities)

The notation u ∼ N(0,Q) defines the process noise as a gaussian distribution with mean zero and covariance Q.

Measurement update - Compares the "predicted" location with the sensor measurement. The predicted location and measured location will provide a updated location. The Kalman filter will put more weight on either the predicted location or the measured location depending on the uncertainty of each value.

We are going to obtain the data from Laser measurements.

*Lidar Measurements* allows us to get the current position but not its velocity. On the other hand is have more resolution. 

![alt text][image5]

Above equation helps us to find x and P

z --> Measurement vector. For a lidar sensor, the z vector contains the *position - x* and *position - y* measurements.

H --> matrix that projects your belief about the bicycle's current state into the measurement space of the sensor. For lidar, this is a fancy way of saying that we discard velocity information from the state variable since the lidar sensor only measures position: The state vector x contains information about [px​,py​,vx​,vy​] whereas the z vector will only contain [px,py]. Multiplying Hx allows us to compare x, our belief, with z, the sensor measurement.

R --> Covariance matrix of the measurement noise.




### Extended Kalman Filter
KF assumes the motion and measurement models are linear and that the state space can be represented by a unimodal Gaussian distribution. Self-driving car will execute non linear motion. Non linear actions will result in non-Gaussian posterior distributions that cannot be properly modeled by a closed form equation. 

EKF approximates motion and measurements to linear functions locally (i.e. by using the first two terms of a Taylor series) to compute a best Gaussian approximation of the posterior covariance. This trick allows EKF to be applied efficiently to a non-linear problems.

![alt text][image6]

### Accuracy

![alt text][image1]

### Reference
[EKF](https://medium.com/intro-to-artificial-intelligence/extended-kalman-filter-simplified-udacitys-self-driving-car-nanodegree-46d952fce7a3)