clear all; clf;
%% DECLARE VARIABLES
%testing Hugh

armlength = 5;
lift_constant = .5; %Denoted as "k" in the technical paper
drag_constant = 1.23; %Denoted as "b" in the technical paper
I = [1 0 0; 0 1 0; 0 0 1]; % Moment of Inertia of drone (diagonal assuming the drone is symmetrical)
rotor_inertia = 3*10^(-5);    %moment of inertia of rotor


% Time Simulation Variables 
starting_time = 0;
ending_time = 10;
time_step = 0.01;
steps = (ending_time - starting_time)/time_step;
time_interval = linspace(starting_time, ending_time, steps);

% Coordinate Variables

euler_angles = zeros(3,steps); % roll, pitch yaw
location = zeros(3,steps); % x y z 
c = [location; euler_angles]; % x,y,z,roll,pitch,yaw, and C is the center of the drone.

c_angular_velocity = zeros(3,steps);
c_angular_acceleration = zeros(3,steps); %Roll, pitch, yaw

c_velocity  = zeros(3,steps);
c_acceleration = zeros(3,steps);

torque_about_C = zeros(3,steps);
rotor_torque = zeros(4, steps);
rotor_av = zeros(4, steps); % Angular velocity of rotors 1 through 4 going counterclockwise
rotor_aa = zeros(4, steps); % Angular accerlation of rotors 1 through 4 going counterclockwise
thrust = zeros(4,steps); % Thrust on rotors 1 through 4 going counterclockwise.



%% INITIAL CONDITIONS


%% CALCULATIONS

% Calculate torque about center

%% Run simulation

%Simulate rotor angular velocity
for t = 1:length(time_interval)    
    
        rotor_av(1,t) = 1;
        rotor_av(2,t) = 0;
        rotor_av(3,t) = 1;
        rotor_av(4,t) = 0;

end


for t = 1:length(time_interval)-1           %just so we can calculate one step ahead and not crash at the end of the loop
    
    
    %Integrate to find rotor angular accleration of rotors 
    for i = 1:4
        rotor_aa(i,t) = (rotor_av(i, t+1) - rotor_av(i,t))/time_step;
    end
    
    
    %Calculate thrusts from angular velocities
    for i = 1:4
        thrust(i, t) = lift_constant * rotor_av(i, t)^2;
    end
    
    %Calculate rotor torques from angular velocities and angular
    %accelerations
    for i = 1:4
        rotor_torque(i, t) = drag_constant * rotor_av(i, t)^2 + rotor_inertia * rotor_aa(i, t);
    end
    
    
    %Calculate net torque
    torque_about_C(1,t) = (thrust(2,t) - thrust(4,t))*armlength; % Affects roll
    torque_about_C(2,t) =  (thrust(3,t) - thrust(1,t))*armlength; % Affects pitch
    torque_about_C(3,t) = sum(rotor_torque(:, t)); %Affects yaw
    
    %Calculate angular acceleration
    c_angular_acceleration(:,t) = inv(I)*torque_about_C(:,t);
    
    %"Integrate" to calculate angular velocity and Euler Angles. This if
    %condition is sketchy, might not be needed
    if t > 1
        c_angular_velocity(:,t) = c_angular_velocity(:,t-1) + c_angular_acceleration(:,t)*time_step;
        euler_angles(:, t) = euler_angles(:, t-1) + c_angular_velocity(:, t)*time_step; 
        euler_angles(:, t) = angle_converter(euler_angles(:,t));
        
        
        %Plot
        cla;
        figure(1)
        plot_3D_stationary_drone([0;0;0], euler_angles(:,t), armlength);
        axis([-6 6 -6 6 -6 6]);
        title(['Plot at time: ' num2str(t) ' out of ' num2str(length(time_interval))] );
        pause(0.1);
    end
end


%% GRAPH RESULTS

%Second component of AA, AV, EA
figure(2)
hold on
plot(time_interval,c_angular_acceleration(2,:));
plot(time_interval,c_angular_velocity(2,:));
plot(time_interval,euler_angles(2,:));
hold off
title('First component of AA, AV, EA')
legend('Angular Accerlation', 'Angular Velocity', 'Euler Angles')


    
