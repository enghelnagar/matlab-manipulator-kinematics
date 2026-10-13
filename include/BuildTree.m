function [iTj_0] = BuildTree()
% This function should build the tree of frames for the chosen manipulator.
% Inputs: 'None'
% Outputs: The tree of frames.

% iTj_0 corresponds to the trasformation from the frame <i> to <i'> which
% for q = 0 is equal to the trasformation from <i> to <i+1>
% (see notes)

% iTj_0 is a 3-dimensional matlab matrix, suitable for defining tree of
% frames. iTj_0 should represent the transformation matrix between the i-th and j-th
% frames. iTj_0(row,col,joint_idx)

% --- Joint 1: Base to Frame 1 (Trans Z 0.105) ---
iTj_0(1,1,1) = 1; iTj_0(1,2,1) = 0; iTj_0(1,3,1) = 0; iTj_0(1,4,1) = 0;
iTj_0(2,1,1) = 0; iTj_0(2,2,1) = 1; iTj_0(2,3,1) = 0; iTj_0(2,4,1) = 0;
iTj_0(3,1,1) = 0; iTj_0(3,2,1) = 0; iTj_0(3,3,1) = 1; iTj_0(3,4,1) = 0.105;
iTj_0(4,1,1) = 0; iTj_0(4,2,1) = 0; iTj_0(4,3,1) = 0; iTj_0(4,4,1) = 1;

% --- Joint 2: Frame 1 to Frame 2 (Rot X -90, Trans Z 0.110) ---
iTj_0(1,1,2) = 0; iTj_0(1,2,2) = 1;  iTj_0(1,3,2) = 0; iTj_0(1,4,2) = 0;
iTj_0(2,1,2) = 0; iTj_0(2,2,2) = 0;  iTj_0(2,3,2) = 1; iTj_0(2,4,2) = 0;
iTj_0(3,1,2) = 1; iTj_0(3,2,2) = 0;  iTj_0(3,3,2) = 0; iTj_0(3,4,2) = 0.110;
iTj_0(4,1,2) = 0; iTj_0(4,2,2) = 0;  iTj_0(4,3,2) = 0; iTj_0(4,4,2) = 1;

% --- Joint 3: Frame 2 to Frame 3 (Rot X 90, Trans X 0.1) ---
iTj_0(1,1,3) = 0; iTj_0(1,2,3) = 0; iTj_0(1,3,3) = 1;  iTj_0(1,4,3) = 0.1;
iTj_0(2,1,3) = 0; iTj_0(2,2,3) = -1; iTj_0(2,3,3) = 0;  iTj_0(2,4,3) = 0;
iTj_0(3,1,3) = 1; iTj_0(3,2,3) = 0; iTj_0(3,3,3) = 0;  iTj_0(3,4,3) = 0;
iTj_0(4,1,3) = 0; iTj_0(4,2,3) = 0; iTj_0(4,3,3) = 0;  iTj_0(4,4,3) = 1;

% --- Joint 4: Frame 3 to Frame 4 (Rot X -90, Trans Z 0.325) ---
iTj_0(1,1,4) = 0; iTj_0(1,2,4) = 0;  iTj_0(1,3,4) = 1; iTj_0(1,4,4) = 0;
iTj_0(2,1,4) = 0; iTj_0(2,2,4) = -1;  iTj_0(2,3,4) = 0; iTj_0(2,4,4) = 0;
iTj_0(3,1,4) = 1; iTj_0(3,2,4) = 0;  iTj_0(3,3,4) = 0; iTj_0(3,4,4) = 0.325;
iTj_0(4,1,4) = 0; iTj_0(4,2,4) = 0;  iTj_0(4,3,4) = 0; iTj_0(4,4,4) = 1;

% --- Joint 5: Frame 4 to Frame 5 (Special Rot, Trans Z 0.095) ---
iTj_0(1,1,5) = 0;  iTj_0(1,2,5) = 0; iTj_0(1,3,5) = 1;  iTj_0(1,4,5) = 0.095;
iTj_0(2,1,5) = -1; iTj_0(2,2,5) = 0; iTj_0(2,3,5) = 0;  iTj_0(2,4,5) = 0;
iTj_0(3,1,5) = 0;  iTj_0(3,2,5) = -1; iTj_0(3,3,5) = 0;  iTj_0(3,4,5) = 0;
iTj_0(4,1,5) = 0;  iTj_0(4,2,5) = 0; iTj_0(4,3,5) = 0;  iTj_0(4,4,5) = 1;

% --- Joint 6: Frame 5 to Frame 6 (Rot Z 180, Trans Z 0.095) ---
iTj_0(1,1,6) = -1; iTj_0(1,2,6) = 0;  iTj_0(1,3,6) = 0; iTj_0(1,4,6) = 0;
iTj_0(2,1,6) = 0;  iTj_0(2,2,6) = -1;  iTj_0(2,3,6) = 0; iTj_0(2,4,6) = 0;
iTj_0(3,1,6) = 0;  iTj_0(3,2,6) = 0;  iTj_0(3,3,6) = 1; iTj_0(3,4,6) = 0.095;
iTj_0(4,1,6) = 0;  iTj_0(4,2,6) = 0;  iTj_0(4,3,6) = 0; iTj_0(4,4,6) = 1;

% --- Joint 7: Frame 6 to Frame 7/EE (Identity Rot, Trans Z 0.445) ---
iTj_0(1,1,7) = 1; iTj_0(1,2,7) = 0;  iTj_0(1,3,7) = 0; iTj_0(1,4,7) = 0;
iTj_0(2,1,7) = 0; iTj_0(2,2,7) = 1;  iTj_0(2,3,7) = 0; iTj_0(2,4,7) = 0;
iTj_0(3,1,7) = 0; iTj_0(3,2,7) = 0;  iTj_0(3,3,7) = 1; iTj_0(3,4,7) = 0.355;
iTj_0(4,1,7) = 0; iTj_0(4,2,7) = 0;  iTj_0(4,3,7) = 0; iTj_0(4,4,7) = 1;

end