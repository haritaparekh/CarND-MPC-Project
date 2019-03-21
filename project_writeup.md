### Model Predictive Control

Simulator provides waypoints of reference trajectory, position of car and velocity. I used code from assignments to start with. After experimenting with it, final model was implemented and appropriate costs were chosen.

### 1. The Model

I used simple kinematic model. This model does not consider interaction of tires with road, tire slip or any other forces. Vehicle state is defined by vehicle position **(*x*, *y*)**, vehicle orientation **(*ψ* - psi)** and velocity **(*v*)**. Cross track error **(*cte*)** as well as orientation error **(*eψ* - epsi)** are also part of vehicle state. Vehicle state is updated based on current state and previous actuations with the help of following model equations:

<br>***x<sub>t+1</sub> = x<sub>t</sub> + v<sub>t</sub>cos(ψ<sub>t</sub>) * dt***
<br>***y<sub>t+1</sub> = y<sub>t</sub> + v<sub>t</sub>sin(ψ<sub>t</sub>) * dt***
<br>***ψ<sub>t+1</sub> = ψ<sub>t</sub> + (v<sub>t</sub>/L<sub>f</sub>)δ<sub>t</sub> * dt***
<br>***v<sub>t+1</sub> = v<sub>t</sub> + a<sub>t</sub> * dt***
<br>***cte<sub>t+1</sub> = f(x<sub>t</sub>) - y<sub>t</sub> + v<sub>t</sub>sin(eψ<sub>t</sub>) * dt***
<br>***eψ<sub>t+1</sub> = ψ<sub>t</sub> - ψdes<sub>t</sub> + (v<sub>t</sub>/L<sub>f</sub>)δ<sub>t</sub> * dt***

Model outputs two actuator values - steering angle **(*δ* - delta)** and throttle value **(*a*)**.

### 2. Timestep Length and Elapsed Duration (N & dt)

MPC tries to optimize state variables for ***N*** timesteps. Time difference between two timesteps is given by ***dt***. As value of ***N*** increases, computational cost increases. On the other hand, lower value of ***N*** does not give enough time for prediction horizon. Similarly, higher value of ***dt*** makes approximations of continuous reference trajectory less accurate. So, depending on how fast you want to go, it is important to choose appropriate values for ***N*** and ***dt***. After multiple trials, I set ***dt=0.1*** which is latency time and ***N=10*** giving prediction horizon of 1 second. 

### 3. Polynomial Fitting and MPC Preprocessing

Waypoints provided by simulator are in simulator coordinate system. These points are first transformed to vehicle coordinate system with the help of following equations:

<br>***x' = x cos(θ) + y sin(θ)***
<br>***y' = -x sin(θ) + y cos(θ)***

These transformed waypoints are then used to fit a 3rd degree polynomial. Reference trajectory is passed to control block as a polynomial.

### 4. Model Predictive Control with Latency

It takes some time to propagate commands through system, typically on the order of 100ms. To simulate this, a delay of 100ms is introduced before MPC results are sent to simulator. So, these actuations are not accurate for the state of vehicle after this delay. To handle this, optimal trajectory is computed starting from the time after latency duration. This is achieved by constraining actuation values to values from previous iteration for latency duration. With this, vehicle model calculates controls for latency duration as well.

### Output

Here is link to my full [video result](output_videos/mpc_output.mp4)

![alt text](output_videos/mpc_output.gif "output_video")

