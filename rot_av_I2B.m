function matrix = rot_av_I2B(euler_angles)

phi = euler_angles(1);
theta = euler_angles(2);
psi = euler_angles(3);

matrix = zeros(3,3);

matrix(1,1) = 1;
matrix(1,2) = 0;
matrix(1,3) = -sin(theta);
matrix(2,1) = 0;
matrix(2,2) = cos(phi);
matrix(2,3) = cos(theta)*sin(phi);
matrix(3,1) = 0;
matrix(3,2) = -sin(phi);
matrix(3,3) = cos(theta)*cos(phi);
end