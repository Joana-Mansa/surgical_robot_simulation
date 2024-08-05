% forwardKinematics.m
function T0 = DirectKinematics(DH)
%
% homogeneous transformation matrices with respect to the arm base frame
% T0 = DirectKinematics(DH)
% Then function T0 takes as input DH paramters
% returns T0, a set of transformations with regards to the base frame
%
%

    % Number of joints
    n = size(DH, 1);
    
    % Initialize the transformation matrix as an identity matrix
    T0 = zeros(4, 4, n);
    T = zeros(4, 4, n); % stores the individual transformations of the joints between consecutive frames
    
    % compute homogeneous transformation matrix(HTM) between consecutive frames
    % according to the DH convention
    % 
    for i = 1:n % indexing from i from 1 to n
        T(:, :, i) = Homogeneous(DH(i, :)); % assign a 4 x 4 HTM to a 3D array T
    end
        
        % Transformation matrix for the current joint
        T0(:, :, 1) = T(:, :, 1); % transformation matrix of the first joint

        for i = 2: n % from the second joint relative to the base frame, multiply the previous joint matrix with the current joint matrix
            T0(:, :, i) = T0(:, :, i-1)*T(:,:,i);
   
        end
end
