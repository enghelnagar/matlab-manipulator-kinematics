%% Template Exam Modelling and Control of Manipulators
clc;
close all;
clear;
addpath('include'); % put relevant functions inside the /include folder 

%% Compute the geometric model for the given manipulator
iTj_0 = BuildTree();
disp('iTj_0')
disp(iTj_0);

% Joint 6 is Prismatic (1), others are Rotational (0)
jointType = [0 0 0 0 0 1 0]; 
geometricModel = geometricModel(iTj_0,jointType);

%% Q1.3
% Compute transformation matrix from base to frame 5 and from frame 2 to frame 6
% We need to update geometry first (using qi from Q1.4 or generic q)
qi = [pi/4, -pi/4, 0, -pi/4, 0, 0.15, pi/4]';
geometricModel.updateDirectGeometry(qi);

bT5 = geometricModel.getTransformWrtBase(5);
bT6 = geometricModel.getTransformWrtBase(6);
bT2 = geometricModel.getTransformWrtBase(2);

% Transformation of Frame 2 with respect to Frame 6: 6_T_2 = inv(b_T_6) * b_T_2
T_6_2 = inv(bT6) * bT2;

disp('bT5 (Base to Frame 5):');
disp(bT5);
disp('6T2 (Frame 2 w.r.t Frame 6):');
disp(T_6_2);


%% Q1.4 Simulation
% Given the following configurations compute the Direct Geometry for the manipulator

% Compute iTj : transformation between the base of the joint <i>
% and its end-effector taking into account the actual rotation/traslation of the joint
qi = [pi/4, -pi/4, 0, -pi/4, 0, 0.15, pi/4]';
geometricModel.updateDirectGeometry(qi)
disp('iTj')
disp(geometricModel.iTj);

% Compute the transformation of the ee w.r.t. the robot base
bTe = geometricModel.getTransformWrtBase(length(jointType));  
disp('bTe')
disp(bTe)

% Show simulation ?
show_simulation = true;

% Set initial and final joint positions
qf = [5*pi/12, -pi/3, 0, -pi/4, 0, 0.18, pi/5]';

%%%%%%%%%%%%% SIMULATION LOOP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation variables
% simulation time definition 
samples = 100;
t_start = 0.0;
t_end = 10.0;
dt = (t_end-t_start)/samples;
t = t_start:dt:t_end; 

pm = plotManipulators(show_simulation);
pm.initMotionPlot(t);

qSteps =[linspace(qi(1),qf(1),samples)', ...
    linspace(qi(2),qf(2),samples)', ...
    linspace(qi(3),qf(3),samples)', ...
    linspace(qi(4),qf(4),samples)', ...
    linspace(qi(5),qf(5),samples)', ...
    linspace(qi(6),qf(6),samples)', ...
    linspace(qi(7),qf(7),samples)'];

% LOOP 
for i = 1:samples

    brij= zeros(3,geometricModel.jointNumber);
    q = qSteps(i,1:geometricModel.jointNumber)';
    % Updating transformation matrices for the new configuration 
    geometricModel.updateDirectGeometry(q)
    % Get the transformation matrix from base to the tool
    bTe = geometricModel.getTransformWrtBase(length(jointType)); 

    %% ... Plot the motion of the robot 
    % Fix: using mod on index to plot periodically
    if (mod(i,10) == 0) 
        for j=1:geometricModel.jointNumber
            bTi(:,:,j) = geometricModel.getTransformWrtBase(j); 
        end
        pm.plotIter(bTi)
    end

end

% Ensure final configuration is plotted
for j=1:geometricModel.jointNumber
    bTi(:,:,j) = geometricModel.getTransformWrtBase(j); 
end
pm.plotFinalConfig(bTi)

%% Q1.5
km = kinematicModel(geometricModel);

% Compute Jacobian of link 6 w.r.t Base for final configuration qf
geometricModel.updateDirectGeometry(qf);
J_link6 = km.getJacobianOfLinkWrtBase(6);

disp('Jacobian of Link 6 w.r.t Base (at qf):');
disp(J_link6);

%% Q1.6
% Compute Jacobian for End-Effector (Link 7) at qf
km.updateJacobian(); % Uses qf from previous step
J_ee = km.J;

disp('Jacobian of End-Effector w.r.t Base (at qf):');
disp(J_ee);


%% Q1.7
% Given q and q_dot, compute velocities projected on EE frame
q_given = [0.7, -0.1, 1, -1, 0, 0.03, 1.3]';
q_dot_given = [0.9, 0.1, -0.2, 0.3, -0.8, 0.5, 0]';

% 1. Update Geometry
geometricModel.updateDirectGeometry(q_given);

% 2. Update Jacobian
km.updateJacobian();
J_base = km.J;

% 3. Compute Twist in Base Frame [v_base; w_base]
twist_base = J_base * q_dot_given;
w_base = twist_base(1:3);
v_base = twist_base(4:6);

% 4. Project on End-Effector Frame
% Need Rotation matrix from Base to EE
bTe_given = geometricModel.getTransformWrtBase(7);
bRe = bTe_given(1:3, 1:3);

% v_e = R^T * v_b
v_ee_frame = bRe' * v_base;
w_ee_frame = bRe' * w_base;

disp('Linear Velocity projected on EE frame (v_e/b)^e:');
disp(v_ee_frame);
disp('Angular Velocity projected on EE frame (w_e/b)^e:');
disp(w_ee_frame);