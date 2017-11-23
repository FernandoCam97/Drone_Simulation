clear all; clf;
%% DECLARE VARIABLES

armlength = 0.225;  %length in m
g = [0; 0; -9.81]; 
mass = 0.45; %Mass in kg
lift_constant = 3*10^(-6); %Denoted as "k" in the paper
drag_constant = 1*10^(-7); %Denoted as "b" in the paper
I = [5*10^(-3) 0 0; 0 5*10^(-3) 0; 0 0 9*10^(-3)]; % Moment of Inertia of drone (diagonal assuming the drone is symmetrical)
rotor_inertia = 3*10^(-5);    %moment of inertia of rotor


% Time Simulation Variables 
starting_time = 0;
ending_time = 10;
time_step = 0.1;
steps = (ending_time - starting_time)/time_step;
time_interval = linspace(starting_time, ending_time, steps);

% Coordinate Variables

euler_angles = zeros(3,steps); % roll, pitch yaw
location = zeros(3,steps); % x y z 
c = [location; euler_angles]; % x,y,z,roll,pitch,yaw, and C is the center of the drone.

c_angular_velocity = zeros(3,steps);
c_angular_acceleration = zeros(3,steps); %Roll, pitch, yaw
nu = zeros(3,steps); % Angular velocities in the body frame

c_velocity  = zeros(3,steps);
c_acceleration = zeros(3,steps);

torque_about_c = zeros(3,steps);
rotor_torque = zeros(4, steps);
rotor_av = zeros(4, steps); % Angular velocity of rotors 1 through 4 going counterclockwise
rotor_aa = zeros(4, steps); % Angular accerlation of rotors 1 through 4 going counterclockwise
thrust = zeros(4,steps); % Thrust on rotors 1 through 4 going counterclockwise.
total_thrust = zeros(3, steps); %Denoted "T" in paper. Note only z component is nonzero 



%% INITIAL CONDITIONS


%% CALCULATIONS

% Calculate torque about center

%% Run simulation

%Simulate rotor angular velocity
for t = 1:length(time_interval)    
    
    if(t<10)
        rotor_av(1,t) = 610+0.1*t;
        rotor_av(2,t) = 610;
        rotor_av(3,t) = 610-0.1*t;
        rotor_av(4,t) = 610;
    elseif(t<20)
        rotor_av(1,t) = 610-0.1*t;
        rotor_av(2,t) = 610;
        rotor_av(3,t) = 610+0.1*t;
        rotor_av(4,t) = 610;
    else
        rotor_av(1,t) = 640;
        rotor_av(2,t) = 640;
        rotor_av(3,t) = 640;
        rotor_av(4,t) = 640;
    end

end


for t = 1:length(time_interval)-1           %just so we can calculate one step ahead and not crash at the end of the loop
    
    
    %Integrate to find rotor angular accleration of rotors 
    for i = 1:4
        rotor_aa(i,t) = (rotor_av(i, t+1) - rotor_av(i,t))/time_step;
    end
    
    
    %Calculate thrust from angular velocities
    for i = 1:4
        thrust(i, t) = lift_constant * rotor_av(i, t)^2;
    end
    total_thrust(3,t) = sum(thrust(:,t));   %Only z component is nonzero 
    
    
    %Calculate rotor torques from rotor angular velocities and angular
    %accelerations
    for i = 1:4
        rotor_torque(i, t) = drag_constant * rotor_av(i, t)^2 + rotor_inertia * rotor_aa(i, t);
    end
    
    
    %Calculate net torque
    torque_about_c(1,t) = (thrust(2,t) - thrust(4,t))*armlength; % Affects roll
    torque_about_c(2,t) = (thrust(3,t) - thrust(1,t))*armlength; % Affects pitch
    torque_about_c(3,t) = rotor_torque(1,t)-rotor_torque(2,t)+rotor_torque(3,t)-rotor_torque(4,t); %Affects yaw
    
    % Calculate nu (angular velocities in the body frame)
    nu(:,t) = solve_diff_nu(torque_about_c,I,time_step,t,location(:,1));
    
    % Calculate angular velocity
    c_angular_velocity(:,t) = get_euler_angles_dot(euler_angles(:,t), nu(:,t));
    
    
    %"Integrate" to calculate angular velocity and Euler Angles. This if
    %condition is sketchy, might not be needed
    if t > 1
        euler_angles(:, t) = euler_angles(:, t-1) + c_angular_velocity(:, t)*time_step; 
        euler_angles(:, t) = angle_converter(euler_angles(:,t));
    
        % Calculate angular acceleration
        c_angular_acceleration(:,t) = get_euler_angles_ddot(euler_angles(:,t),...
        nu(:,t), I, torque_about_c(:,t));
        
        % Calculate (translational) acceleration
        c_acceleration(:,t) = g + (1/mass)*(rot_frame_B2I(euler_angles(:,t))...
            *total_thrust(:,t)); 
        
        c_velocity(:,t) = c_velocity(:, t-1) + c_acceleration(:,t)*time_step;
        location(:,t) = location(:,t-1) + c_velocity(:,t)*time_step;
        
        
        % Live plot
        cla;
        figure(1)
        plot_3D_stationary_drone(location(:,t), euler_angles(:,t), armlength);
        axis([-4 4 -4 4 -4 4]);
        title(['Plot at time: ' num2str(t) ' out of ' num2str(length(time_interval))] );
        pause(0.1);
        
    end
end


%% GRAPH RESULTS

%Second component of AA, AV, EA
%{
figure(2)
hold on
plot(time_interval,c_angular_acceleration(2,:));
plot(time_interval,c_angular_velocity(2,:));
plot(time_interval,euler_angles(2,:));
hold off
title('First component of AA, AV, EA')
legend('Angular Accerlation', 'Angular Velocity', 'Euler Angles')
%}

    
