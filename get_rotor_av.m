function rotor_av = get_rotor_av(armlength, lift_constant, drag_constant, ...
    thrust, tau)

rotor_av = zeros(4,1);

tau_phi = tau(1);
tau_theta = tau(2);
tau_psi = tau(3);

l = armlength;
k = lift_constant;
b = drag_constant;

rotor_av(1) = sqrt(abs(thrust/(4*k) - tau_theta/(2*k*l) - tau_psi/(4*b)));
rotor_av(2) = sqrt(abs(thrust/(4*k) - tau_phi/(2*k*l) + tau_psi/(4*b)));
rotor_av(3) = sqrt(abs(thrust/(4*k) + tau_theta/(2*k*l) - tau_psi/(4*b)));
rotor_av(4) = sqrt(abs(thrust/(4*k) + tau_phi/(2*k*l) + tau_psi/(4*b)));
end
