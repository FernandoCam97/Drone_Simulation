function tau_c = get_tau_c(k_phi_d, k_phi_p, k_theta_d, k_theta_p, k_psi_d, ...
    k_psi_p, des_euler_angles_dot, des_euler_angles, euler_angles_dot,...
    euler_angles, I) 
% Note that location and angles are the current values (ie not the whole
% matrix)

% Set up variables
tau_c = zeros(3,1);

phi_d_dot = des_euler_angles_dot(1);
theta_d_dot = des_euler_angles_dot(2);
psi_d_dot = des_euler_angles_dot(3);

phi_dot = euler_angles_dot(1);
theta_dot = euler_angles_dot(2);
psi_dot = euler_angles_dot(3);

phi_d = des_euler_angles(1);
theta_d = des_euler_angles(2);
psi_d = des_euler_angles(3);

phi = euler_angles(1);
theta = euler_angles(2);
psi = euler_angles(3);

Ixx = I(1,1);
Iyy = I(2,2);
Izz = I(3,3);

% Compute equations

tau_c(1) = (k_phi_d*(phi_d_dot - phi_dot) + k_phi_p*(phi_d - phi))*Izz;;
tau_c(2) = (k_theta_d*(theta_d_dot - theta_dot) + k_theta_p*...
    (theta_d - theta))*Iyy;
tau_c(3) = (k_psi_d*(psi_d_dot - psi_dot) + k_psi_p*(psi_d - psi))*Ixx;

end