# Write up

[image1]: ./images/model.png "Model"
[image2]: ./images/actuator.png "Model"

## Model

The chosen model has a six-dimensional state vector whereas the first four describe the dynamic behavior of the car where there is the position in x and y direction as well as the car`s orientation and velocity. The remaining two state vector entries contain the Cross Track Error and Orientation Error of the car.  
Alltogether, the state parameters are updated with the Kinematic model from the classroom:  

![alt text][image1]

As actuators the car`s steering angle and throttle/brake is chosen which leads to a two-dimensional vector.
The contraints of the two actuator parameters is chosen to be:

![alt text][image2]

This ensures bounded values for the steering and the acceleration/deceleration.

## Parameters

As hyperparameters `N=10` nodes on the predicted trajectory are chosen with a time difference of `dt = 0.1`. The value for `N` was chosen because higher `N` would only end up to a higher computational time but would not improve the predicted trajectory and a smaller value would only consider the near future of the car and would e.g. not plan with an upcoming curve.  
The value for `dt` was chosen as a good trade-off for having a reactive manner so that this finer resolution plans in smaller steps. Bigger values for `dt` would degrade the performance by not planning the route in much detail whereas smaller value would shorten the time horizon. Especially for high velocities, the product of `N` and `dt` must be sufficiently big to plan the self driving's car route in enough advance.

## Waypoints

The waypoints of the car's trajectory first have to be transformed. Therefore, the input coordinates in global frame are first shifted in by substracting the ego car's position. Then, these values are rotated around psi to finally obtain relative coordinates in the car frame.  
After that, the car frame coordinates are used to fit the polynomial.

## Latency

To deal with latency the update equations are placed after fitting the polynomial which involves two steps.
First, the CTE and EPSI are calculated for the car's pose in `px=0`,
`py=0,`psi=0` which can be found in line 114-118 of `main.cpp`.

// Fill in state vector  
Eigen::VectorXd state(6);  
double cte = polyeval(coeffs, 0);  
double epsi = - atan(coeffs[1]);  
state << 0, 0, 0, v, cte, epsi;  

Second, the previous actuations are chosen for all other timestamps than the next one to address the latency which can be found in line 111-115 of `MPC.cpp`:

// Use previous actuations to account latency
if (t > 1) {   
	a = vars[a_start + t - 2];
	delta = vars[delta_start + t - 2];
}

Then, all state variables are updated according to the previous mentioned model.