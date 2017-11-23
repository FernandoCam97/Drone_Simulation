function nu_dot = get_dnu_dt(nu_at_t, I, torque_about_c_at_t)

nu_dot = zeros(3,1);

p = nu_at_t(1);
q = nu_at_t(2);
r = nu_at_t(3);

Ixx = I(1,1);
Iyy = I(2,2);
Izz = I(3,3);

tau_phi = torque_about_c_at_t(1);
tau_theta = torque_about_c_at_t(2);
tau_psi = torque_about_c_at_t(3);

nu_dot(1) = (Iyy-Izz)*q*r/Ixx + tau_phi/Ixx;
nu_dot(2) = (Izz-Ixx)*p*r/Iyy + tau_theta/Iyy;
nu_dot(3) = (Ixx-Iyy)*p*q/Izz + tau_psi/Izz;

end