% MATLAB controller for Webots
% File:          my_controller.m
% Date:
% Description:
% Author:
% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
%desktop;
%keyboard;

TIME_STEP = 64;

% get and enable devices, e.g.:
% camera = wb_robot_get_device('camera');
%  wb_camera_enable(camera, TIME_STEP);
%   motor = wb_robot_get_device('motor');

% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the loop when Webots signals the termination
%
while wb_robot_step(TIME_STEP) ~= -1
 camera = wb_robot_get_device('camera');
 front_left_led = wb_robot_get_device("front left led");
 front_right_led = wb_robot_get_device("front right led");
 imu = wb_robot_get_device("inertial unit");
 gps = wb_robot_get_device("gps");
 compass = wb_robot_get_device("compass");
 gyro = wb_robot_get_device("gyro");
 camera_roll_motor = wb_robot_get_device("camera roll");
 camera_pitch_motor = wb_robot_get_device("camera pitch");
 wb_camera_enable(camera, TIME_STEP);
 wb_inertial_unit_enable(imu, TIME_STEP);
 wb_gps_enable(gps, TIME_STEP);
 wb_compass_enable(compass, timestep);
 wb_gyro_enable(gyro, timestep);
 wb_keyboard_enable(timestep);
 
 front_left_motor = wb_robot_get_device("front left propeller");
 front_right_motor = wb_robot_get_device("front right propeller");
 rear_left_motor = wb_robot_get_device("rear left propeller");
 rear_right_motor = wb_robot_get_device("rear right propeller");
  motors = [front_left_motor, front_right_motor, rear_left_motor, rear_right_motor];
  % read the sensors, e.g.:
  %  rgb = wb_camera_get_image(camera);

  % Process here sensor data, images, etc.

  % send actuator commands, e.g.:
  %  wb_motor_set_postion(motor, 10.0);

  % if your code plots some graphics, it needs to flushed like this:
  drawnow;

end

% cleanup code goes here: write data to files, etc.
