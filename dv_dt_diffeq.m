function dv_dt = dv_dt_diffeq(t, v, euler_angles, drag_coeff, g, ...
    total_thrust, time_interval, mass)

phi = euler_angles(1,:);
theta = euler_angles(2,:);
psi = euler_angles(3,:);

phi = interp1(time_interval, phi, t);
theta = interp1(time_interval, theta, t);
psi = interp1(time_interval, psi, t);

thrust = interp1(time_interval, total_thrust(3,:), t);

% Part of the eqn. Calculated once to simplify and in order to easily
% isolate a particular component of it

thrust_inertial = (1/mass)*rot_frame_B2I([phi; theta; psi])*[0; 0; thrust];

dv_dt = zeros(3,1);

dv_dt(1) = thrust_inertial(1)-(1/mass)*drag_coeff(1,1)*v(1);
dv_dt(2) = thrust_inertial(2)-(1/mass)*drag_coeff(2,2)*v(2);
dv_dt(3) = g(3) + thrust_inertial(3)-(1/mass)*drag_coeff(3,3)*v(3);


end