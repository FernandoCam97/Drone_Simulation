function nu_i = solve_diff_nu(torques,I,time_step,iteration,...
    initial_conditions)

Ixx = I(1,1);
Iyy = I(2,2);
Izz = I(3,3);

diff_end_time = iteration*time_step;
time_interval = linspace(0,diff_end_time, iteration);
torques = torques(:,1:iteration);

%Ensures that we are on at least the second iteration. This is required
%because in order to interpolate the torque values, we need at least two
if iteration ~= 1
    
    %Syntax for ode45 is: ode45(function, time interval, initial
    %conditions)
    [t, nu] = ode45(@(t,nu)...
        dnu_dt_diffeq(t, nu, time_interval,torques,Iyy,Ixx,Izz),...
        [0 diff_end_time], initial_conditions);

else
    dnu_dt = @(t,nu) [(Iyy-Izz)*nu(2)*nu(3)/Ixx + torques(1,1)/Ixx;...
        (Izz-Ixx)*nu(1)*nu(3)/Iyy + torques(2,1)/Iyy;...
        (Ixx-Iyy)*nu(1)*nu(2)/Izz + torques(3,1)/Izz];
    
    [t, nu] = ode45(dnu_dt, [0 diff_end_time], initial_conditions);
end

%Outputs the "most recent" nu value. Thus nu_i is a 3x1 column vector 
nu_i = nu(end,:)';
end

