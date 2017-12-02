function thrust_c = get_thrust_c(mass, g, k_z_p, k_z_d, des_location, ...
    des_location_dot, location, c_velocity, euler_angles)
% Note that location and angles are the current values (ie not the whole
% matrix)

phi = euler_angles(1);
theta = euler_angles(2);

% Taking transpose of matrix so we convert from inertial to body frame
rot_mat = rot_frame_B2I(euler_angles)';

des_location = rot_mat*des_location;
des_location_dot = rot_mat*des_location_dot;
location = rot_mat*location;
c_velocity = rot_mat*c_velocity;



% NB: this equation is the same as appears in the paper, except here, g is
% negative

thrust_c = (-g(3) + k_z_d*(des_location_dot(3) - c_velocity(3)) + k_z_p*(...
    des_location(3) - location(3)))*mass/(cos(phi)*cos(theta));

end
