%% Geometric Model Class - GRAAL Lab
classdef geometricModel < handle
    % iTj_0 is an object containing the trasformations from the frame <i> to <i'> which
    % for q = 0 is equal to the trasformation from <i> to <i+1> = >j>
    % (see notes)
    % jointType is a vector containing the type of the i-th joint (0 rotation, 1 prismatic)
    % jointNumber is a int and correspond to the number of joints
    % q is a given configuration of the joints
    % iTj is  vector of matrices containing the transformation matrices from link i to link j for the input q.
    % The size of iTj is equal to (4,4,numberOfLinks)
    properties
        iTj_0
        jointType
        jointNumber
        iTj
        q
    end

    methods
        % Constructor to initialize the geomModel property
        function self = geometricModel(iTj_0,jointType)
            if nargin > 1
                self.iTj_0 = iTj_0;
                self.iTj = iTj_0;
                self.jointType = jointType;
                self.jointNumber = length(jointType);
                self.q = zeros(self.jointNumber,1);
            else
                error('Not enough input arguments (iTj_0) (jointType)')
            end
        end
        function updateDirectGeometry(self, q)
            %%% GetDirectGeometryFunction
            % This method update the matrices iTj.
            % Inputs:
            % q : joints current position ;

            % The function updates:
            % - iTj: vector of matrices containing the transformation matrices from link i to link j for the input q.
            % The size of iTj is equal to (4,4,numberOfLinks)
            
            % Update the internal q property
            self.q = q;

            for k = 1:self.jointNumber
                % Get the fixed transformation for this link (at q=0)
                T_fixed = self.iTj_0(:,:,k);
                
                if self.jointType(k) == 0 
                    % --- Rotational Joint (0) ---
                    % Rotation around local Z-axis
                    theta = q(k);
                    Rz = [cos(theta) -sin(theta) 0 0;
                          sin(theta)  cos(theta) 0 0;
                          0           0          1 0;
                          0           0          0 1];
                    
                    % Update local transform: Fixed * Motion
                    self.iTj(:,:,k) = T_fixed * Rz;
                    
                elseif self.jointType(k) == 1 
                    % --- Prismatic Joint (1) ---
                    % Translation along local Z-axis
                    d = q(k);
                    Tz = [1 0 0 0;
                          0 1 0 0;
                          0 0 1 d;
                          0 0 0 1];
                    
                    % Update local transform: Fixed * Motion
                    self.iTj(:,:,k) = T_fixed * Tz;
                end
            end
        end
        function [bTk] = getTransformWrtBase(self,k)
            %% GetTransformatioWrtBase function
            % Inputs :
            % k: the idx for which computing the transformation matrix
            % outputs
            % bTk : transformation matrix from the manipulator base to the k-th joint in
            % the configuration identified by iTj.

            % Initialize as Identity matrix
            bTk = eye(4);
            
            % Validate input k
            if k < 1 || k > self.jointNumber
                error('Joint index k out of bounds');
            end
            
            % Chain multiplication from Base (0) up to Frame (k)
            % bTk = 0T1 * 1T2 * ... * (k-1)Tk
            for j = 1:k
                bTk = bTk * self.iTj(:,:,j);
            end
        end

    end
end