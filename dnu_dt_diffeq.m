function dnu_dt = dnu_dt_diffeq(t, nu, time_interval, torques, Iyy, Ixx, Izz)
torques_phi = torques(1,:);
torques_theta = torques(2,:);
torques_psi = torques(3,:);

torques_phi = interp1(time_interval, torques_phi, t);
torques_theta = interp1(time_interval, torques_theta, t);
torques_psi = interp1(time_interval, torques_psi, t);

dnu_dt = zeros(3,1);
dnu_dt(1) = (Iyy-Izz)*nu(2)*nu(3)/Ixx + torques_phi./Ixx;
dnu_dt(2) = (Izz-Ixx)*nu(1)*nu(3)/Iyy + torques_theta./Iyy;
dnu_dt(3) = (Ixx-Iyy)*nu(1)*nu(2)/Izz + torques_psi./Izz;
end