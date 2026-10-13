%% Kinematic Model Class - GRAAL Lab
classdef kinematicModel < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        J % Jacobian
    end
    methods
        % Constructor to initialize the geomModel property
        function self = kinematicModel(gm)
            if nargin > 0
                self.gm = gm;
                self.J = zeros(6, self.gm.jointNumber);
            else
                error('Not enough input arguments (geometricModel)')
            end
        end
        function bJi = getJacobianOfLinkWrtBase(self, i)
            %%% getJacobianOfJointWrtBase
            % This method computes the Jacobian matrix bJi of joint i wrt base.
            % Inputs:
            % i : joint indnex ;
            % The function returns:
            % bJi
            
            %TO DO
            
            % Initialize Jacobian matrix for this specific link
            bJi = zeros(6, self.gm.jointNumber);
            
            % 1. Get the transformation of the target link 'i' to find its origin position
            bT_target = self.gm.getTransformWrtBase(i);
            P_target = bT_target(1:3, 4); 
            
            % 2. Loop through all joints 'j' from 1 up to 'i'
            for j = 1:i
                % Get transformation of the CURRENT joint 'j'
                % Note: Using CURRENT frame based on the specific Geometric convention used
                bT_curr = self.gm.getTransformWrtBase(j);
                
                % Extract joint axis (K_j) and position (P_j) from CURRENT frame
                K_j = bT_curr(1:3, 3);     % Rotational axis (Z-axis of frame j)
                P_j = bT_curr(1:3, 4);     % Origin of frame j
                
                if self.gm.jointType(j) == 0  
                    % --- Rotational joint ---
                    % Angular part: K_j
                    % Linear part: K_j x (P_target - P_j)
                    J_w = K_j;
                    J_v = cross(K_j, P_target - P_j);
                    
                elseif self.gm.jointType(j) == 1  
                    % --- Prismatic joint ---
                    % Angular part: 0
                    % Linear part: K_j
                    J_w = [0; 0; 0];
                    J_v = K_j;
                end
                
                % Assemble column j: [J_w; J_v] (Angular Velocity First)
                bJi(:, j) = [J_w; J_v];
            end
        end
        function updateJacobian(self)
        %% Update Jacobian function
        % The function update:
        % - J: end-effector jacobian matrix
            % TO DO
            
            % Simply call the generic function for the last link (End-Effector)
            % jointNumber corresponds to the index of the final link/frame
            self.J = self.getJacobianOfLinkWrtBase(self.gm.jointNumber);
            
        end
    end
end 