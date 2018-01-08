function des_thrust = get_des_thrust(mass, des_d_xi, des_euler_angles_at_t)

phi = des_euler_angles_at_t(1);
theta = des_euler_angles_at_t(2);
psi = des_euler_angles_at_t(3);

dx = des_d_xi(1);
dy = des_d_xi(2);
dz = des_d_xi(3);
g = 9.81;

t1 = dx*(sin(theta)*cos(psi)*cos(phi) + sin(psi)*sin(phi));
t2 = dy*(sin(theta)*sin(psi)*cos(phi) - cos(psi)*sin(phi));
t3 = (dz + g)*cos(theta)*cos(phi);

des_thrust = mass*(t1 + t2 + t3);

end