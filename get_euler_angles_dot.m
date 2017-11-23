function ea_dot = get_euler_angles_dot(euler_angles, nu_at_t)

ea_dot = zeros(3,1);

ea_dot = inv(rot_av_I2B(euler_angles))*nu_at_t;

end