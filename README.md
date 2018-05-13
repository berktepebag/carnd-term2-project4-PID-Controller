## PID Control Project

### Self-Driving Car Engineer Nanodegree Program

### Proportional-Integral-Derivative (PID) Controller

Proportional-Integral-Derivative (PID) Controller, controls the machine according to some error, an error which is calculated according to expected (middle of the lane in our situation) and measured (distance from middle of the lane, CTE).

PID, includes 3 parts:

1. #### Proportional:

Proportional_Error = - CTE * K_proportional

Basicly, proportional moves the car in the opposite direction of error (CTE) mulitiplied with a coefficient. 

In the main.cpp, there is a type of twiddle I have implemented which looks for CTE and if it is lower than best_CTE for two step time increases K_proportinal. If it is higher than best_CTE for 2 step time decreases K_proportional. With every change done to K_proportional, amount of time step (distance) car will go before reseting the track increases. By the this way, we can take all the situations (left-right turns, straight parts, etc.) and find a K_proportional after completing a full track.

Tried running with K_proportional = 1.0, K_integral = 0, K_derivative = 0 throttle = 0.1 . K_proportional settled around 0.1. Fixed K_proportional to 0.1. 

<img width="800" alt="After 17 Trials Kp settles around 0.1" src="/imgs/PID_Project_Finding_Kp.JPG">

2. #### Integral:

Integral_Error = - CTE * total_CTE * K_integral

If error is not going to zero in a certain time, total error will increase so our steering angle will increase too. This will cause oscillation and eventually car goes out of track. Since the track is counter clockwise, total error will grow. For every 1000 time step, divided total error by 2 to avoid huge increase in the error.

Kept K_integral really small because total error increases after few time steps and if K_Integral is not small enough dominates the total error which causes a high steering angle. 0.001 to 0.0001 is fine enough to smoothen the steering angle. 

3. #### Derivative:

Derivative_error = - (CTE - prev_CTE) * K_derivative

Derivative, in this case looks for the difference between last time step CTE and current CTE (time step or delta time = 1). If difference is high, steering angle will increase to put car back on the track. ie. Car turns a sharp left, which causes CTE to increase suddenly. Since difference will be high, steering angle will also increase with it.

Tried K_derivative:1.0 but after sharp turns it was driving car out of the track. Then tried 0.5, it was not steering enough this time. Settled between two which is fine enough. 

#### Final Thoughts:

After settling down with Kp= 0.1, Ki= 0.0001, Kd = 0.8 to increase the speed of car implemented SetThrottle, an if else statement where car looks for CTE and speed, decides the throttle. If speed and CTE is high, car is not stable slows down the car. If it is stable, increases the throttle and decreases Kp, because if you are moving faster you do not want to turn with sharp steering angles. If car is very stable (i.e straight part) increase throttle more to achieve higher speeds. But it also causes a problem where a sharp turn comes after straight part. Since car does not know a sharp corner is coming (Conv. NET can fix this problem easily.), enters the turn without slowing, CTE increases dramaticly and oscillation starts. If car was above certain speed, it cannot turn as intended and goes out of the track. 

Project can be improved further by implementing a better SetThrottle which decides not wiht if-else statement but according to formulized relation between CTE and speed.








