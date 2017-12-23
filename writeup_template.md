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

The MPC procedure follows a similar structure to what was taught in the lessons. First, the current state and waypoints at time t are given. Using the current state (x,y,psi,velocity,steering angle), the state at time t+1 is predicted. The waypoints are transformed into the coordinate space of the car's predicted state at time t+1. A polynomial is fitted onto the transformed waypoints. The orientation error and cte are calculated from the fitted polynomial's distance from the car and slope at x=0. The values of state t+1's x, y, psi, and velocity are fed into the MPC solver along with the orientation error, cte, and polynomial's coefficients. The MPC solver uses an IPOpt optimizer to return the steering angles and throttle values of the next N timesteps of time dt that optimizes the orientation error and cte. I used N=7 and dt=0.1. The model formulas are similar to the model formulas in the lecture, except I included acceleration in the estimate to fing the x and y placements. In the constraints, I set acceleration to be within range of [-40,40]; all other constraints are similar to the lectures. For the cost functtion, I used the same costs given in the MPC-to_Line quiz; I added parameters to increase the effect of different values. For example, the costs of the differential of the yaw rates and acceleration were multiplied by 30000 and 2500, respectively. This was supposed to steadily increase the acceleration and steadily increase the turning angle, similar to what would be observed when actually driving a car. I also changed the reference velocity to 40 mph.
The solver returns the estimations of the next N states, steering angles, and acceleration values. I use only the first given acceleration, velocity, and steering angle. I used multiple simulations to better understand the effect of velocity on throttle. From my observance, velocity would have a 0.0113 effect on throttle. The acceleration value is divided by 40 and added to the throttle value. The steering angle is divided by 25 degrees (in radians) to get the steer value. 

#### 2. Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

My initial perception of how to choose N was wrong. I based my intuition of choosing N similar to how a person would need to judge N timesteps. Although a person may need to judge 30 timesteps into the future, the computer needs far less. A value of 7 for N proved to be sufficient. Values that were too high proved to damage the model by attempting to correct for observations too far into the future. dt was chosen to be equivalent to the latency. 

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

My initial approach was to use the code given in the mpc_to_line solution to complete the problem. The code in the mpc_to_line solution is basic, and there were many changes I made to make this code work. When finding the next state, the mpc lectures only provide a model that assumes a yaw rate of 0. I used an additional acceleration term to the position equations. I believe this would give a better prediction of the next state. I predicted 100ms into the future before fitting the polynomial or using the solver to account for latency. I set N=7, dt=.1s, and ref_v=40mph. This means I was using the same timespan as latency to predict .7 seconds into the future of a vehicle traveling at a reasonable speed. I added hyperparameters to some cost functions. I added most of these hyperparameters while attempting to figure out why the model wouldn't correctly behave. I multiplied the steering angle by -1 to account for the inverted directions. The instructions recommended I do this. I changed acceleration to have bounds of -40 and 40. The car's acceleration should not have a bound of [-1,1]. Most cars get to 60mph(27m/s) in 4-7 seconds. After the MPC solver is used, the next steering angles and accelerations are fed into the model. The acceleration is converted into a throttle measurement by adding a proportion of the velocity to the acceleration divided by 40. I did a quick search on the internet for an actual throttle equation, but I realized there would be more parameters involved in a complete throttle value equation, such as the weight and horsepower. I settled with using this, because it apears to work. The steering value is obtained by dividing the steering angle by 25 degrees. 
Where this model fails is when the reference velocity is set higher. The hyperparameters affecting the cost are not optimized, and the used throttle equation will not provide a completely accurate throttle value. What I could do to make this more robust is tune the hyperparameters. 
