clear; matlabrc; clc; close all;
addpath(genpath('lib'))
addpath(genpath('src'))
rng(0) % For repeatability

%% Settings
% Simulation time settings:
dt = 1;
duration = 30*60;
tspan = dt:dt:duration;
jd = greg2jd(2019,1,1,0,0,0);

% Satellite physical settings:
wheelSpeeds = [0;0;0];
J = eye(3);
AmR = 0;

% Initial Position/Velocity:
mu = 3.986004418e14;
a = 6778000;
e = 0;
inc = deg2rad(52);
omega = deg2rad(0);
Omega = deg2rad(0);
theta = deg2rad(-90);
[r,v] = kep2rv(a,e,inc,omega,Omega,theta,mu);

% Initial attitude:
[q,w] = EarthTargeter2(r,v,mu);
q = a2q(q2a(q)*e2a(5*randn(3,1)));
w = w + .01*randn(3,1);

% Satellite initialize:
sat = Spacecraft(r,v,q,w,mu,'wheelSpeeds',wheelSpeeds,'J',J,'AmR',AmR,'jd',jd,'integrator','rk4');

% Setup camera:
cam.focal = 3.60; %(mm)
cam.sensor = [3.76,2.74]; %(mm)
cam.body_attitude = e2a(200,0,0);
% cam.resolution = [2592,1944];
cam.resolution = floor(300*cam.sensor);

%% Propagate simulation:
num_steps = length(tspan);
data = zeros(num_steps,7);
attitudes = zeros(num_steps,4);
angRates = zeros(num_steps,3);
positions = zeros(num_steps,3);
velocities = zeros(num_steps,3);
gps_meas = zeros(num_steps,6);
gyro_meas = zeros(num_steps,3);
for ii = 1:num_steps
    % Get state estimate:
    estQuat    = sat.attitude;
    estAngRate = sat.angRate;
    
    % Get target state:
    [desQuat, desAngRate] = EarthTargeter2(sat.position, sat.velocity, sat.mu);
    
    % Run control:
    kp = .1;
    kd = .1;
    wheelTorques = control(estAngRate, desAngRate, estQuat, desQuat, sat.J, kp, kd);
    
    % Propagate dynamics:
    sat.propagate(dt,wheelTorques);
    
    % Calculate local euler angles:
    [~,q_error] = quatError(estQuat,desQuat);
    A_error = q2a(q_error);
    pitch = acosd(A_error(3,3));
    roll  = acosd(A_error(1,1));
    disp(['angRate: ',num2str(rad2deg(estAngRate')), ' | roll: ',num2str(roll),' | pitch: ', num2str(pitch)])
    %disp(sat.wheelSpeeds')
    
    % Store results:
    cam_attitude = a2q(cam.body_attitude*q2a(sat.attitude));
    cam_attitude = [cam_attitude(4); cam_attitude(1:3)];
    data(ii,:) = [sat.position', cam_attitude'];
    attitudes(ii,:) = sat.attitude';
    angRates(ii,:) = sat.angRate';
    positions(ii,:) = sat.position';
    velocities(ii,:) = sat.velocity';
    gps_meas(ii,:) = [sat.position',sat.velocity'];
    gyro_meas(ii,:) = sat.angRate';
end

% Write simulation data to matrix:
csvwrite('data/pose_history2.csv',data)
csvwrite('data/camera_data2.csv',[cam.resolution,cam.sensor,cam.focal])

% Save truth data:
save('data/truth2','attitudes','angRates','positions','velocities')
save('data/cam2','cam')
save('data/gps2','gps_meas')
save('data/gyro2','gyro_meas')