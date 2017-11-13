function angle_pi = angle_converter(euler_angles)
% Converts to angles in (-pi, pi]
    angle_pi = zeros(3,1);
    for i = 1 : 3
        angle_pi(i) = mod(euler_angles(i) + pi, 2*pi) - pi;
        if angle_pi(i) == -pi
            angle_pi(i) = pi;
        end
    end
end