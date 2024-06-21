%%%%%%%%%%%%%%%%%%%%%%%%%
% Guillem Cornella
% 3 DOF Robot kinematics
% BioRobotics Lab, UCI
%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; clc;  close all;

% Initialize joint angular positions
q4 = 0;
q5 = 0;
q6 = 0;

q_DH = [q4, q5, q6]
q_URDF = [q4, q5, q6];

% Upload urdf
r = importrobot('Assembly_v12/urdf/Assembly_v12.urdf'); % Change to your directory
r.DataFormat='row';

T_urdf = getTransform(r, q_URDF, "link_3")

% Extract position and orientation from the end-effector transform
position = T_urdf(1:3, 4); % XYZ position
orientation = T_urdf(1:3, 1:3); % Orientation as rotation matrix

% Convert orientation to quaternion for easier handling if needed   
orientationQuaternion = rotm2quat(orientation);

% Plot from URDF only
figure,
show(r, q_URDF);

% Using DH parameters
d4 = 0.056;
d5 = 0;
d6 = 0.3;
L1 = Link('d', d4 , 'a', 0, 'alpha', -pi/2);    % Define first link
L2 = Link('d', d5, 'a', 0, 'alpha', pi/2);      % Define second link
L3 = Link('d', d6, 'a', 0, 'alpha', 0);         % Define third link

% Now we need to join these to create a serial-link 3DOF robot manipulator
bot = SerialLink([L1 L2 L3], 'name', 'my robot');

% Plot from DH only
figure,
bot.plot(q_DH);
view(140, 30)

% Forward kinematics 
T_DH = bot.fkine(q_DH)

%% Modeling
% https://link.springer.com/chapter/10.1007/978-1-84628-642-1_2
% https://www.andre-gaschler.com/rotationconverter/

%joint variables
syms qi
syms q4
syms q5
syms q6
assume(qi,'real')
assume(q4,'real')
assume(q5,'real')
assume(q6,'real')

%link lengths (all axis intersect at the same point)
syms ai
assume(ai,'real')

%link offsets
syms di
syms d4
syms d6
assume(di,'real')
assume(d4,'real')
assume(d6,'real')

%link twists
syms alphai alpha4 alpha5
assume(alphai,'real')

% MODELLING
% A matrices
A01i = [cos(qi) -sin(qi)*cos(alphai)  sin(qi)*sin(alphai) ai*cos(qi);...
       sin(qi)  cos(qi)*cos(alphai) -cos(qi)*sin(alphai) ai*sin(qi);...
          0     sin(alphai)          cos(alphai)         di;...
          0    0     0    1];
    
A01 = subs(A01i,{ai,di,alphai,qi},{0, d4, -pi/2, q4});
A12 = subs(A01i,{ai,di,alphai,qi},{0, 0,  pi/2, q5});
A23 = subs(A01i,{ai,di,alphai,qi},{0, d6, 0, q6});

%% FORWARD KINEMATICS
Tsyms = A01*A12*A23

% Particularize
vd4 = 0.056;
vd6 = 0.3;
vT = subs(Tsyms,{d4, d6},{vd4, vd6})

% % Extract the rotation part of the transformation matrix

%% Double VERIFICATION USING THE ROBOTICS TOOLBOX FOR MATLAB
% Using DH parameters
L1 = Link('d', 0.056 , 'a', 0, 'alpha', -pi/2); % Define first link
L2 = Link('d', 0, 'a', 0, 'alpha', pi/2);       % Define second link
L3 = Link('d', 0.3, 'a', 0, 'alpha', 0);        % Define third link
bot = SerialLink([L1 L2 L3], 'name', 'my robot');

qa = [0.5,0.5,0.5];
T1 = bot.fkine(qa)

format short
vT03 = double(subs(vT,{q4,q5,q6},{qa}))


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FORWARD KINEMATICS
% Given the joint angles q3, q4 and q5 we can determine the pose of the robot's
% end-effector

% Our robot is capable of turning at 0.094deg per step
max_steps_per_turn = 3808;                  % int64(360/0.094);
gearbox_reduction = max_steps_per_turn/8    % 476 steps per turn
quarter_step = gearbox_reduction/4          % 119 steps per quarter turn

th4 = zeros(1, quarter_step*2);
th5 = zeros(1, quarter_step*2);
th6 = zeros(1, quarter_step*2);

%% * Maximum range of motion for all joints (UNIFORM)
% A quarter step is pi/2, then we divide it by quarter step (119) to get
% the 0.0132
th4 = [0:0.0132:pi/2];
th4 = [th4, th4(end):-0.0132:0];
th5 = [0:-0.0132:-pi/2];
th5 = [th5, th5(end):0.0132:0];
th6 = [zeros(1, quarter_step), pi*ones(1, quarter_step)];  

% Plot one position of the robot to show the end effector's axis
figure
i=119;
bot.plot([th4(i) th5(i) th6(i)]); 
view(140, 30)
title(['q4: ',num2str(th4(i)), ' - q5: ',num2str(th5(i)), ' - q6: ',num2str(th6(i)), ' - theta: ',num2str(0)]) %th5(i) th6(i) angle_degrees

%% TRAJ (peak at 90 and skewed)
th4 = [0:0.0132/2:pi/4];
th4 = [th4, th4(end)*ones(1, 119)];
th5 = [0:-0.0132:-pi/2];
th5 = [th5, th5(end):0.0132:0];
th6 = [0:0.0132*2:pi]; 
th6 = [th6, th6(end):-0.0132*2:0];

%% Shuffle cards (uniform)
th4 = [0:0.0132:pi/2];
th4 = [th4, th4(end):-0.0132:0];
th5 = [-pi/2*ones(1, quarter_step*2)];
th6 = [pi:-0.0132*2:0]; 
th6 = [th6, th6(end):0.0132*2:pi];


%% Biceps extension (peaked at 90deg)
th4 = [0:0.0132:pi/2];
th4 = [0*ones(1, quarter_step), th4, th4(end):-0.0132:0];
th5 = [0:-0.0132:-pi/2];
th5 = [th5, -pi/2*ones(1, quarter_step), th5(end):0.0132:0];
th6 = [pi/2:0.0132:pi, pi:-0.0132*2:0,  0:0.0132:pi/2 ];  

%% Cup stacking (highly peaked at 90 deg)
th4 = [pi/2:-0.0132:0];
th4 = [th4, th4(end):0.0132:pi/2];

th5 = [-pi/2:0.0132:-pi/4];
th5 = [th5, th5(end):-0.0132:-pi/2];
th5 = [th5, -pi/2*ones(1,quarter_step)];

th6 = [ pi/3*ones(1,quarter_step*2)];

%% Kind of normal distribution
% Define parameters
mu = pi/2;       % Mean of the distribution
sigma = pi/6;    % Standard deviation of the distribution
num_samples = 2380; % Number of samples to generate
% Generate random numbers from the normal distribution
data = normrnd(mu, sigma, num_samples, 1)

th4 = [zeros(1,quarter_step*2)];
th5 = [-pi/2*ones(1,quarter_step*2)];
th6 = sort(data)';

%% Handshake
th4 = [zeros(1,quarter_step*2)];

th5 = [-pi/2:0.0132:-pi/4];
th5 = [th5, th5(end):-0.0132:-pi/2];
th5 = [th5, -pi/2*ones(1,quarter_step)];

th6 = [ pi/2*ones(1,quarter_step*2)];

%% Visualize the robot movements in the URDF format or the DH format
figure
data = [];
iters = 0;
while iters < 3 % repeat 3 times
    iters = iters+1
    % iterate through all the possible joint angles at the same time
    for i = 1:length(th4)
        % Draw the robot
        show(r, [th4(i) th5(i) -th6(i)]);
      
        % Calculate the pose of the robot end-effector as an SE(3) homogeneous transformation (4x4) for the joint configuration q (1xN).
        q = [th4(i) th5(i) th6(i)];
        TK = bot.fkine(q);

        % Get the rotation matrix
        R = TK.R;
        
        % Get the euler angles
        eulZYX = rotm2eul(R,"ZYX");

        % Rotate the vector original vector of the wrist x direction
        v = [1; 0; 0];
        A = R * v;

        % A = T.t;       % Get the coordinates of the End-Effector (the tip of the robot)
        B = [0 0 1];   % The normal direction (End-Effector pointing upwards). Defined as Z here.
        
        % Calculate the angle between A and B (the tilt angle)
        angle_radians = acos(dot(A, B) / (norm(A) * norm(B)));
        % angle_radians2 = acos(A(3) / sqrt(A(1)^2+A(2)^2+A(3)^2))

        % Convert the angle from radians to degrees
        angle_degrees = rad2deg(angle_radians);
        
        % Get a list with the joint angles and the tilt angle
        data = [data;  [th4(i), th5(i), th6(i), angle_degrees]];
        [th4(i) th5(i) th6(i) angle_degrees];

        % Plot the position of the robot, if you uncomment the following 3
        % lines, comment line show(r, [th4(i) th5(i) -th6(i)]); 
        % both together are incompatible
        % bot.plot([th4(i) th5(i) th6(i)]); 
        % view(140, 30)
        % title(['q4: ',num2str(th4(i)), ' - q5: ',num2str(th5(i)), ' - q6: ',num2str(th6(i)), ' - theta: ',num2str(angle_degrees)]) %th5(i) th6(i) angle_degrees

        % Delay the movement of the robot for visualization purposes
        pause(0.1)
    end
end

% save('Data.mat', "data");

json_str = jsonencode(data(:,4));    % Save to json file
file_name = 'data2.json';        % Define the file name
fileID = fopen(file_name, 'w'); % Open the file for writing
fprintf(fileID, '%s', json_str);% Write the JSON string to the file
fclose(fileID);                 % Close the file

histogram(data(:,4),18)
kurtosis(data(:,4))