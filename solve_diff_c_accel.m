function v_i = solve_diff_c_accel(g, mass, euler_angles, ...
    total_thrust, drag_coeff, iteration, initial_conditions, time_step)
% v_i is c_velocity (ie. velocity of the center of the drone in the 
% inertial frame) at the "ith" time step


diff_end_time = iteration*time_step;
time_interval = linspace(0,diff_end_time, iteration);

total_thrust = total_thrust(:,1:iteration);
euler_angles = euler_angles(:,1:iteration);
%Ensures that we are on at least the second iteration. This is required
%because in order to interpolate the torque values, we need at least two
if iteration ~= 1
    
    %Syntax for ode45 is: ode45(function, time interval, initial
    %conditions)
    [t, v] = ode45(@(t,v) dv_dt_diffeq(t, v, euler_angles, drag_coeff, g, ...
        total_thrust, time_interval, mass),...
        [0 diff_end_time], initial_conditions);

else
    dv_dt = @(t,v) g + (1/mass)*(rot_frame_B2I(euler_angles(:,1))...
        *total_thrust(:,1)) - (1/mass)*drag_coeff*v; 
    
    [t, v] = ode45(dv_dt, [0 diff_end_time], initial_conditions);
end

%Outputs the "most recent" v value. Thus v_i is a 3x1 column vector 
v_i = v(end,:)';


end