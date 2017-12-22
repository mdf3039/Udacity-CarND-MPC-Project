## Writeup Template

### You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---

**MPC Controller Project**

The goals / steps of this project are the following:

* Your code should compile. Code must compile without errors with cmake and make.
* Student describes their model in detail. This includes the state, actuators and update equations.
* Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.
* A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.
* The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.
* No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle). The car can't go over the curb, but, driving on the lines before the curb is ok.


## [Rubric](https://review.udacity.com/#!/rubrics/896/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Writeup / README  

You're reading it!


### Compilation

#### 1. Your code should compile. Code must compile without errors with cmake and make.

After the last push to GitHub, I compiled the code to ensure there were not any errors before submitting for review.


### Implementation

#### 1. Student describes their model in detail. This includes the state, actuators and update equations.

The MPC procedure follows a similar structure to what was taught in the lessons. First, the current state and waypoints at time t are given. Using the current state (x,y,psi,velocity,steering angle), the state at time t+1 is predicted. The waypoints are transformed into the coordinate space of the car's predicted state at time t+1. A polynomial is fitted onto the transformed waypoints. The orientation error and cte are calculated from the fitted polynomial's distance from the car and slope at x=0. The values of state t+1's x, y, psi, and velocity are fed into the MPC solver along with the orientation error, cte, and polynomial's coefficients. The MPC solver uses an IPOpt optimizer to return the steering angles and throttle values of the next N timesteps of time dt that optimizes the orientation error and cte. I used N=30 and dt=0.1. The model formulas are similar to the model formulas in the lecture, except I used the next state formula given in the Kalman Filter lectures instead of the the state formula given in the MPC lecture to determine x and y at timesteps. The formula given in the Kalman Filter gain lectures takes into account the yaw rate when approximating the position of x/y at the next timestep. In the constraints, I set acceleration to be within range of [-40,40]; all other constraints are similar to the lectures. For the cost functtion, I used the same costs given in the MPC-to_Line quiz; I added parameters to increase the effect of different values. For example, the costs of the differential of the yaw rates and acceleration were multiplied by 2500. This was supposed to steadily increase the acceleration and steadily increase the turning angle, similar to what would be observed when actually driving a car. I also changed the reference velocity to 20 mph.
The solver returns the estimations of the next N states, steering angles, and acceleration values. I use only the first given acceleration and steering value. The acceleration value is divided by 40 to get the throttle value. The steering value is divided by 25 degrees (in radians) to get the steer value. 

#### 2. Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

The N and dt are chosen based on how far you believe the solver must observe down the road to make good judgements. At low speeds, there is not much need to look ahead, but at high speeds, a turn needs to be forseen many meters ahead to project the best steering angle. My car has a reference velocity of 20mph=8.9m/s, so there is not too much necessity to look far ahead. I felt 3 seconds ahead would be sufficient. I also set the dt=0.1s, since the latency is 100 ms = 0.1s. This means N=3/0.1=30. While attempting to get the model to work at higher speeds, I tried multiple values of N ranging from 30-200.

#### 3. A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

A polynomial of the 3rd degree is fitted onto the waypoints. Using the current state (x,y,psi,velocity,steering angle), the state at time t+1 is predicted. The waypoints are transformed into the coordinate space of the car's predicted state at time t+1. A polynomial is fitted onto the transformed waypoints. The orientation error and cte are calculated from the fitted polynomial's distance from the car and slope at x=0.

#### 4. The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

Latency is given as 100ms. The model handles a 100ms latency by predicting 100ms into the future before feeding the state into the MPC solver.


### Simulation

#### 1. No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle). The car can't go over the curb, but, driving on the lines before the curb is ok.

The final model has a vehicle that drives successfully around the track, without hitting any ledges or unsafe surfaces.


---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your model likely fail?  What could you do to make it more robust?

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the model might fail and how I might improve it if I were going to pursue this project further.  

My initial approach was to use the code given in the mpc_to_line solution to complete the problem. The code in the mpc_to_line solution is basic, and there were many changes I made to make this code work. When finding the next state, the mpc lectures only provide a model that assumes a yaw rate of 0. I used the equations given in the Kamlan Filter lectures. I believe his would give a better prediction of the next state. I predicted 100ms into the future before fitting the polynomial or using the solver to account for latency. I set N=30, dt=.1s, and ref_v=20mph. This means I was using the same timespan as latency to predict 3 seconds into the future of a vehicle traveling at a erasonable speed. I added hyperparameters to some cost functions. I added most of these hyperparameters while attempting to figure out why the model wouldn't correctly behave; they are in no way optimal. I multiplied the steering angle by -1 to account for the inverted directions. The instructions recommended I do this. I changed acceleration to have bounds of -40 and 40. The car's acceleration should not have a bound of [-1,1]. Most cars get to 60mph(27m/s) in 4-7 seconds. After the MPC solver is used, the next steering angles and accelerations are fed into the model. The acceleration is converted into a throttle measurement by dividing by 40. This is in no way how an actual throttle value should be derived to achieve a desired acceleration; the throttle value would be some function of acceleration and current velocity. I did a quick search on the internet for such an equation, but I realized there would be more parameters involved in a complete throttle value equation, such as the weight and horsepower. I settled with using this, because it works for this speed. The steering value is obtained by dividing the steering angle by 25 degrees. 
Where this model fails is when the reference velocity is set too high. The hyperparameters affecting the cost are not optimized, and the used throttle equation will not provide an accurate throttle value. What I could do to make this more robust is tune the hyperparameters and add a velocity component to the throttle equation, so the throttle value = a*velocity + b*acceleration, where there are now the hyperparameters a and b. But this would also require tuning more parameters. 
