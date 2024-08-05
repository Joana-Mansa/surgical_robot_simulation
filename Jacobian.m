function J = Jacobian(DH)
% Jacobian matrix relates the joint velocities to the end-effector's linear
% and angular velocities
%
% Jacobian takes the DH parameters (nx4) as input
% outputs a J matrix 6 x n
%
%
n = size(DH, 1); % number of joints
J = zeros(6, n); % initialise Jacobian matrix
p = zeros(3, n); % position vectors of each frame
z = zeros(3, n); % z-axes unit vectors of each frame

% computes the HTM from base frame
T0 = zeros(4, 4, n); % stores the cumulative TM for each joint
for i =1:n
    T_i = Homogeneous(DH(i, :)); % compute HT for the i-th joint
    if i ==1 % for the first joint, 
        T0(:, :, i) = T_i; % TM is assigned to T_i
    else
        T0(:, :, i) = T0(:, :, i -1)*T_i; % computes the cumulative transformation matrix
    end 
    p(:, i) = T0(1:3, 4, i); % stores the position of the i-th frame
    z(:, i) = T0(1:3, 3, i); % stores the z-axis in the i-th frame

end

% initialise base frame variables
z0 = [0 0 1]';
p0 = [0 0 0 ]';
J(:, 1) = [cross(z0, p(:,n)-p0);
    z0];
for i=2:n % compute Jacobian for each joint
    J(:, i) = [cross(z(:, i-1), p(:, n)-p(:, i-1));
        z(:,i-1)];
end 