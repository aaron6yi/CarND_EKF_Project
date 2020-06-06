# Extended Kalman Filter Project 

----

This project utilizes a kalman filter to estimate the state of a moving object with lidar & radar measurements.

#### Usage

1. Navigate to `CarND-Extended-Kalman-Filter-Project` and run the commands below

```
./install-ubuntu.sh
mkdir build && cd build
cmake .. && make
./ExtendedKF
```

2. Open the Simulator after running the `./ExtendedKF `command
3. Click on "SELECT" 
4. Click on "START" 

#### My project includes the following files:

- `FusionEKF.cpp`
- `FusionEKF.h`
- `FusionEKF.cpp`
- `FusionEKF.h`
- `json.hpp`
- `kalman_filter.cpp` 
- `kalman_filter.h` 
- `main.cpp`
- `measurement_package.h`
- `tools.cpp`
- `tools.h`
- `writeup.md`

#### Program structure:

The main flow is in `main.cpp`, and the algorithm implementation is in:

`FusionEKF.cpp`, values initialization, and flow of "Measurement", "Prediction", Sensor fusion.

`KalmanFilter.cpp`, implement Kalman filter for Lidar and Radar data.

`tools.cpp`, calculate `Jacobian` and `RMSE`.

#### Extended Kalman Filter V.S. Kalman Filter

![](https://raw.githubusercontent.com/aaron7yi/CarND_EKF_Project/master/3.png)

**Kalman Filter**

- *x* is the mean state vector.
- *F* is the state transition function.
- *P* is the state covariance matrix, indicating the uncertainty of the object's state.
- *u* is the process noise, which is a Gaussian with zero mean and covariance as Q.
- *Q* is the covariance matrix of the process noise.

------

**Extended Kalman Filter **
- *y* is the the difference between the measurement and the prediction. In order to compute it, The state vector transform to measurement space by measurement function, then compare the measurement and prediction directly.
- *S* is the predicted covariance.
- *H* is the measurement function.
- *z* is the measurement.
- *R* is the covariance matrix of the measurement noise.
- *I* is the identity matrix.
- *K* is the Kalman filter gain.
- *Hj* and *Fj* are the jacobian matrix.

**Both Kalman filters have the same three steps:**

1. First measurement
2. Initialization
3. Prediction
4. Update

#### Illustration:

`KalmanFilter.cpp` illustrated (taken from udacity course video):

![](https://raw.githubusercontent.com/aaron7yi/CarND_EKF_Project/master/4.png)

`FusionEKF.cpp` illustrated (taken from udacity course video):

![](https://raw.githubusercontent.com/aaron7yi/CarND_EKF_Project/master/5.png)



#### Results

**Final result with dataset 1:**

![](https://raw.githubusercontent.com/aaron7yi/CarND_EKF_Project/master/1.png)

**Final result with dataset 2:**

![](https://raw.githubusercontent.com/aaron7yi/CarND_EKF_Project/master/2.png)

#### Recommendation
I'm highly suggest other students utilize the Sensor Fusion formula sheet while watching the video course. You would find it helps you better understanding these linear & non-linear equation from  KF & EKF.