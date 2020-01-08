**Extended Kalman Filter**

 <p align="center">
    <img src="/media/EKF_DEMO.gif" alt="result"
    title="result"  />
</p>

Lidar measurements are red circles, radar measurements are blue circles with an arrow pointing in the direction of the observed angle, and estimation markers are green triangles.



For Setup & Dependencies please refer to official [udacity repository](https://github.com/glaumarkus/CarND-Extended-Kalman-Filter-Project).

---

The goal of this project:
* compute 2D position and velocity of moving objects with Radar / Lidar Data

Assumptions:
- constant velocity

 <p align="center">
    <img src="/media/program_architecture.PNG" alt="result"
    title="result"  />
</p>

The extended kalman filter contained in this repository is able to process LIDAR and RADAR data with a sensor fusion to predict the most likely position after Δt. This performance of the generated prediction is measured within the script with RMSE as opposed to the actual location of the vehicle. 

Performance:

| Measure | RMSE |
|-------|---------|
| px | 0.0973 |
| py | 0.0855 |
| vx | 0.4513 |
| vy | 0.4399 |

Relevant files in this repository:
- data/obj_pose-laser-radar-synthetic-ukf-input.txt
- src/kalman_filter.cpp
- src/FusionEKF.cpp
- src/tools.cpp

The input file for this project contains all observations in chronological order. The Input file features the observation as well as the ground truth of the position (meas & gt). 

| Laser | meas_px | meas_py | timestamp        | gt_px | gt_py | gt_vx    | gt_vy |
|-------|---------|---------|------------------|-------|-------|----------|-------|
| L     | 8.45    | 0.25    | 1477010443349642 | 8.45  | 0.25  | -3.00027 | 0     |

| Radar | meas_rho | meas_phi  | meas_rho_dot | timestamp        | gt_px | gt_py | gt_vx    | gt_vy |
|-------|----------|-----------|--------------|------------------|-------|-------|----------|-------|
| R     | 8.60363  | 0.0290616 | -2.99903     | 1477010443349642 | 8.45  | 0.25  | -3.00027 | 0     |


Due to seperate input from the sensors, they will have to be combined by calculating 2D position and velocity (px,py,vx,vy). Whereas speed and position can be calculated with a single RADAR measurement, the initial LIDAR image holds no information about the approximate velocity. Only when a second measurement is taken, the delta in position will hold information about the movement. 

The kalman filter functions are described in kalman_filter.cpp. As seen in the programm architecture the filter features a predict function as well as seperate update functions for RADAR and LIDAR. RADAR data needs to be converted to match x,y - positional data from rho, phi and rhodot.

The FusionEKF features the main processing function that takes measurements as input and runs the model architecture. The first measurement serves as initialization to set up the relevant matrices for further input. With each upcoming observation a prediction with Δt is run based on the last calculated px,py,vx,vy. Afterwards the state of the matrices is updated.

Tools.cpp features a basic implementation of RMSE calculation, as well as calculation of a Jacobian matrice. 

