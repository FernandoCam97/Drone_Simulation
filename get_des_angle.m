function des_angle = get_des_angle(psi, des_d_xi,type)
% If type == 1, value is theta, 
% otherwise value is phi
dx = des_d_xi(1);
dy = des_d_xi(2);
dz = des_d_xi(3);
g = 9.81;

if type == 1
    des_angle = atan((dx*cos(psi) + dy*sin(psi))/(dz + g));
else
    des_angle = asin((dx*sin(psi) - dy*cos(psi))/(dx^2 + dy^2 + (dz + g)^2));
end


end