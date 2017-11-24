function matrix = get_d_dt_rot_av_B2I(euler_angles, nu_at_t)
%returns what is denoted in the paper as d/dt W inv

euler_angles_dot = get_euler_angles_dot(euler_angles, nu_at_t);

phi = euler_angles(1);
theta = euler_angles(2);
psi = euler_angles(3);

phi_dot = euler_angles_dot(1);
theta_dot = euler_angles_dot(2);

matrix = zeros(3,3);

matrix(1,1) = 0;
matrix(1,2) = phi_dot*cos(phi)*tan(theta)+theta_dot*sin(phi)/(cos(theta))^2;
matrix(1,3) = -phi_dot*sin(phi)*cos(theta)+theta_dot*cos(phi)/(cos(theta))^2;
matrix(2,1) = 0;
matrix(2,2) = -phi_dot*sin(phi);
matrix(2,3) = -phi_dot*cos(phi);
matrix(3,1) = 0;
matrix(3,2) = (phi_dot*cos(phi)/cos(theta))+(phi_dot*sin(phi)*tan(theta)/cos(theta));
matrix(3,3) = -phi_dot*sin(phi)/cos(theta)+theta_dot*cos(phi)*tan(theta)/cos(theta);


end