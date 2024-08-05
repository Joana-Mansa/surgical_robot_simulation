function e = Rot2Quat(R)
% from rotation matrix to quaternions
%
%takes a rotation matrix
% outputs a quaternion
%

e = zeros(4,1); % initialises a 4x1 vector to store the quaternion components

% calculates the scaler part of the quaternion, fourth component
e(4) = .5*sqrt(R(1,1)+R(2,2)+R(3,3)+1); 

% computes the vector part of the quaternion
if (R(3,2)-R(2,3))>=0
    e(1) = .5*sqrt(R(1,1)-R(2,2)-R(3,3)+1);
else
    e(1) = -.5*sqrt(R(1,1)-R(2,2)-R(3,3)+1);
end

if (R(1,3)-R(3,1))>=0
    e(2) = .5*sqrt(-R(1,1)+R(2,2)-R(3,3)+1);
else
    e(2) = -.5*sqrt(-R(1,1)+R(2,2)-R(3,3)+1);
end

if (R(2,1)-R(1,2))>=0
    e(3) = .5*sqrt(-R(1,1)-R(2,2)+R(3,3)+1);
else
    e(3) = -.5*sqrt(-R(1,1)-R(2,2)+R(3,3)+1);
end

% ensure real values and normalise the quaternion
e=real(e);
e = e/norm(e);
