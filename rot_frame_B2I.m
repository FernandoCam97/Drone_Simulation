function matrix = rot_frame_B2I(euler_angles)
%converts from body frame to inertial frame

phi = euler_angles(1);
theta = euler_angles(2);
psi = euler_angles(3);

matrix = zeros(3,3);
matrix(1,1) = cos(psi)*cos(theta);
matrix(1,2) = cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(phi);
matrix(1,3) = cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi);
matrix(2,1) = sin(psi)*cos(theta);
matrix(2,2) = sin(psi)*sin(theta)*sin(phi) + cos(psi)*cos(phi);
matrix(2,3) = sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi);
matrix(3,1) = -sin(theta);
matrix(3,2) = cos(theta)*sin(phi);
matrix(3,3) = cos(theta)*cos(phi);


end

