# Extended-Kalman-Filter
I utilized an Extended Kalman Filter to estimate and track the state of a moving object based on lidar and radar measurements fusion.
The given lidar measurement data is the position of the object (px and py) and the corresponding timestamps; The given radar measurement data is the range, angle and range rate of the object and the corresponding timestamps.
the output is the estimate state of the object (px,py,vx,vy).

The EKF can divided into three steps (initialization, prediction, update)
1. initializing the initial state with the first sensor measurements and kalman filter variables,
then the car will receive another sensor measurement after a time period Δt
2. predicting the state of the object after Δt, in this project for motion model the velocity is assumed as constant during Δt.
3. the prediction information and the measurement information are combined to give the updated state. The Kalman filter will put more weight on either the predicted state or the measured state depending on the uncertainty of each value.
In my case, radar measurements involve a nonlinear measurement function, because the radar measurements is in polar coordinates. So the EKF is used to linearize the nonlinear function using Taylor series expansion and taking the Jacobian(first order derivative of multi-dimensional equations) of the nolinear measurement function.

Then the prediction and update steps repeat themselves in a loop.
