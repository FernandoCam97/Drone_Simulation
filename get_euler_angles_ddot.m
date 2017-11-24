function eta_ddot = get_euler_angles_ddot(euler_angles, nu_at_t, I,...
    torque_about_c_at_t)

eta_ddot = zeros(3,1);

nu_dot = get_dnu_dt(nu_at_t, I, torque_about_c_at_t);
d_dt_w_inv = get_d_dt_rot_av_B2I(euler_angles,nu_at_t);
w_inv = inv(rot_av_I2B(euler_angles));
eta_ddot = d_dt_w_inv*nu_at_t + w_inv*nu_dot;

end