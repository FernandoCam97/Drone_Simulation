function rotor_av = get_rotor_av(armlength, lift_constant, drag_constant, ...
    thrust_c, tau_c)

rotor_av = zeros(4,1);

tau_phi = tau_c(1);
tau_theta = tau_c(2);
tau_psi = tau_c(3);

l = armlength;
k = lift_constant;
b = drag_constant;

% NB if any of these equations (neglecting the abs()) is negative, it might
% correspond to weird stuff we should take into account

rotor_av(1) = sqrt(abs(thrust_c/(4*k) - tau_theta/(2*k*l) - tau_psi/(4*b)));
rotor_av(2) = sqrt(abs(thrust_c/(4*k) - tau_phi/(2*k*l) + tau_psi/(4*b)));
rotor_av(3) = sqrt(abs(thrust_c/(4*k) + tau_theta/(2*k*l) - tau_psi/(4*b)));
rotor_av(4) = sqrt(abs(thrust_c/(4*k) + tau_phi/(2*k*l) + tau_psi/(4*b)));
end
