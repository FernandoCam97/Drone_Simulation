clear all;
clc;
close all;

%% DECLARE VARIABLES
armlength = 0.225;  %length in m. Denoted as "l" in the paper
g = [0; 0; -9.81]; 
mass = 0.45; %Mass in kg
lift_constant = 3*10^(-6); %Denoted as "k" in the paper
drag_constant = 1*10^(-7); %Denoted as "b" in the paper
I = [5*10^(-3) 0 0; 0 5*10^(-3) 0; 0 0 9*10^(-3)]; % Moment of Inertia of drone (diagonal assuming the drone is symmetrical)
rotor_inertia = 3*10^(-5);    %moment of inertia of rotor


% Time Simulation Variables 
starting_time = 0;
ending_time = 10; % Potentially inputted from GUI
time_step = 0.1;
steps = floor((ending_time - starting_time)/time_step);    % Convert to int since steps needs to be int for array indicies
time_interval = linspace(starting_time, ending_time, steps);

% Coordinate Variables
euler_angles = zeros(3,steps); % roll, pitch yaw. Denoted "phi, theta, psi" in paper
location = zeros(3,steps); % x y z 
c = [location; euler_angles]; % x,y,z,roll, pitch,yaw, and C is the center of the drone.

% Motion Variables
c_angular_velocity = zeros(3,steps);
c_angular_acceleration = zeros(3,steps); %Roll, pitch, yaw
nu = zeros(3,steps); % Angular velocities in the body frame

c_velocity  = zeros(3,steps);
c_acceleration = zeros(3,steps);

rotor_av = zeros(4, steps); % Angular velocity of rotors 1 through 4 going counterclockwise
rotor_aa = zeros(4, steps); % Angular accerlation of rotors 1 through 4 going counterclockwise

% Force Variables
torque_about_c = zeros(3,steps);
rotor_torque = zeros(4, steps);
thrust = zeros(4,steps); % Thrust on rotors 1 through 4 going counterclockwise.
total_thrust = zeros(3, steps); %Denoted "T" in paper. Note only z component is nonzero 

% Control Variables 
location_error = zeros(3, steps);       %difference between ideal path and actual location
des_location = zeros(3,steps);      %Planned, ideal path 
des_location_dot = zeros(3,steps);

thrust_control = zeros(1, steps);
tau = zeros(3, steps);      % Denoted in the paper as "tau_phi", "tau_theta", "tau_psi"
des_euler_angles = zeros(3,steps);
des_euler_angles_dot = zeros(3,steps);


% Control Constants 
k_z_d = 2.5;
k_phi_d = 1.75;
k_theta_d = 1.75;
k_psi_d = 1.75;
k_z_p = 1.5;
k_phi_p = 6;
k_theta_p = 6;
k_psi_p = 6;


%% INITIAL CONDITIONS
location(:,1) = [0;0;0];
euler_angles(:,1) = [0;0;0];


%% Pre-Simulation Calculations
for t = 1:length(time_interval)  
    
    % Set desired locations and angles (for control)
    if t>50
        des_euler_angles(:,t) = [pi/16;0;0];
    end
    
    % Calculate velocities of desired variables
    if t > 1
        des_euler_angles_dot(:,t) = (des_euler_angles(:,t) - ...
            des_euler_angles(:,t-1))/time_step;
        des_location_dot(:,t) = (des_location(:,t) - des_location(:,t-1))...
            /time_step;
    end
    %{
    %Simulate rotor angular velocity
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
    %}
    
end


%% Run simulation

for t = 1:length(time_interval)-1           %Just so we can calculate one step ahead and not crash at the end of the loop
        
    
    % Integrate to find rotor angular accleration of rotors 
    for i = 1:4
        rotor_aa(i,t) = (rotor_av(i, t+1) - rotor_av(i,t))/time_step;
    end
    
    
    % Calculate thrust from angular velocities
    for i = 1:4
        thrust(i, t) = lift_constant * rotor_av(i, t)^2;
    end
    total_thrust(3,t) = sum(thrust(:,t));   %Only z component is nonzero 
    
    
    % Calculate rotor torques from rotor angular velocities and angular
    %accelerations
    for i = 1:4
        rotor_torque(i, t) = drag_constant * rotor_av(i, t)^2 + rotor_inertia * rotor_aa(i, t);
    end
    
    
    % Calculate net torque
    torque_about_c(1,t) = (thrust(2,t) - thrust(4,t))*armlength; % Affects roll
    torque_about_c(2,t) = (thrust(3,t) - thrust(1,t))*armlength; % Affects pitch
    torque_about_c(3,t) = rotor_torque(1,t)-rotor_torque(2,t)+rotor_torque(3,t)-rotor_torque(4,t); %Affects yaw
    
    % Calculate nu (angular velocities in the body frame)
    nu(:,t) = solve_diff_nu(torque_about_c,I,time_step,t,location(:,1));
    
    % Calculate angular velocity
    c_angular_velocity(:,t) = get_euler_angles_dot(euler_angles(:,t), nu(:,t));
    
    
    % "Integrate" to calculate angular velocity and Euler Angles. This if
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
        
        
        % CONTROL SECTION START
        
        % Set rotor speeds
        rotor_av(:,t+1) = get_rotor_av(armlength, lift_constant, ...
            drag_constant,...
            ...
            get_thrust_c(mass, g, k_z_p, k_z_d, des_location(:,t), ...
            des_location_dot(:,t), location(:,t), c_velocity(:,t),...
            euler_angles(:,t)), ...
            ...
            get_tau_c(k_phi_d, k_phi_p, k_theta_d, k_theta_p, k_psi_d, ...
            k_psi_p, des_euler_angles_dot(:,t), des_euler_angles(:,t),...
            get_euler_angles_dot(euler_angles(:,t), nu(:,t)),...
            euler_angles(:,t), I));
       
        
        
        % CONTROL SECTION END
        
        
        % Live plot. If condition for ease of toggle
        if true
        cla;
        figure(1);
        %axes(handles.main_plot); %GUI 
        plot_3D_stationary_drone(location(:,t), euler_angles(:,t), armlength);  %GUI included handles here
        %hold on
        plot_path(location(:,1:t)); %GUI included handles here
        axis([-4 4 -4 4 -4 4]);
        title(['Plot at time: ' num2str(t) ' out of ' num2str(length(time_interval))] );
        pause(0.05);
        end
        
    end
    
    % Pause and reset conditions from GUI
    %{
    if get(handles.pause,'UserData') == 1
        while(1)
            pause(0.001);
            if get(handles.pause, 'UserData') == 0
                break;
            end
        end
    end
    if get(handles.reset,'UserData') == true
        set(handles.reset, 'UserData', false);
        break;
    end
    %}
end

%% GRAPH RESULTS

%figure(3)
%plot(time_interval, location);
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


    
