function thrust_c = get_thrust_c(mass, g, k_z_p, k_z_d, des_location, ...
    des_location_dot, location, c_velocity, euler_angles)
% Note that location and angles are the current values (ie not the whole
% matrix)

phi = euler_angles(1);
theta = euler_angles(2);

thrust_c = (g + k_z_d*(des_location_dot - c_velocity) + k_z_p*(...
    des_location - location))*mass/(cos(phi)*cos(theta));


end
