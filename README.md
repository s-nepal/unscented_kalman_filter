# The Unscented Kalman Filter

This is a C++ implementation of the Unscented Kalman Filter. Dummy data for the project is located inside data/obj_pose-laser-radar-synthetic-input.txt. The columns in the data file represent sensor_type, rho_measured, phi_measured, rhodot_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth in that order for Radar data. The columns represent sensor_type, x_measured, y_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth in that order for LiDAR data.

This implementation of the Unscented Kalman Filter uses a sensor fusion approach to use incoming data (either from LiDAR or radar) to predict the location and speed (on an xy plane) of a pedestrian.

To Compile:<br />
mkdir build<br />
cd build<br />
cmake ..<br />
make<br />

To Run:<br />
./UnscentedKF ../data/obj_pose-laser-radar-synthetic-input.txt output.txt
<br />
where "../data/obj_pose-laser-radar-synthetic-input.txt" and "output.txt" are the names of the input data file and the output data file respectively
