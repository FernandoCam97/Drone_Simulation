function nu_i = solve_diff_nu(torques,I,time_step,iteration,...
    initial_conditions)
% Initial Conditions is a 3 x 1 vector with the initial nu.
Iyy = I(2,2);
Ixx = I(1,1);
Izz = I(3,3);
diff_end_time = iteration*time_step;
time_interval = linspace(0,diff_end_time, iteration);
torques = torques(:,1:iteration);
if iteration ~= 1 % Only used since interpolation is done in getode, and we can't do this with one sample point
    [t, nu] = ode45(@(t,nu) getode(t, nu, time_interval,torques,Iyy,Ixx,Izz),...
        [0 diff_end_time], initial_conditions);
else
    dnu_dt = @(t,nu) [(Iyy-Izz)*nu(2)*nu(3)/Ixx + torques(1,1)/Ixx;...
        (Izz-Ixx)*nu(1)*nu(3)/Iyy + torques(2,1)/Iyy;...
        (Ixx-Iyy)*nu(1)*nu(2)/Izz + torques(3,1)/Izz];
    
    [t, nu] = ode45(dnu_dt, [0 diff_end_time], initial_conditions);
end


nu_i = nu(end,:)'; % This will output the first nu as a column vector
end

