function plot_3D_stationary_drone(location, euler_angles, armlength)
    % where c = (x,y,z,euler_angles)
    % Assume that C is at (x,yz) = (0,0,0) for now change
    % later!!
    
    %% Calculate rotor locations
    rotors = zeros(3,4); % Positions of rotors (x,y,z)

    
    rotors(:,1) = rotation_matrix(euler_angles)*[armlength; 0; 0] + location;
    rotors(:,2) = rotation_matrix(euler_angles)*[0; armlength; 0] + location;
    rotors(:,3) = rotation_matrix(euler_angles)*[-armlength; 0; 0] + location;
    rotors(:,4) = rotation_matrix(euler_angles)*[0; -armlength; 0] + location;
    
    %% Plot
    figure(1)
    scatter3(location(1), location(2), location(3));     %plot center point
    for i = 1:4
        figure(1)
        hold on
        scatter3(rotors(1,i), rotors(2,i), rotors(3,i), 'filled', ...
            'MarkerFaceColor', 'k'); % Plots points of rotors
    end
    
    for i = 1:2 %plot lines connecting rotors
        figure(1)
        hold on
        plot3(linspace(rotors(1,i),rotors(1,i+2),100),...
            linspace(rotors(2,i),rotors(2,i+2),100), ...
            linspace(rotors(3,i),rotors(3,i+2),100),'r', 'LineWidth', 2);
    end
    
    plot_3D_orientation(location,euler_angles,armlength); % Plots orientation of centre point
    
    xlabel('x');
    ylabel('y');
    zlabel('z');
    text(rotors(1,1), rotors(2,1), rotors(3,1), ' 1');
    text(rotors(1,2), rotors(2,2), rotors(3,2), ' 2');
    text(rotors(1,3), rotors(2,3), rotors(3,3), ' 3');
    text(rotors(1,4), rotors(2,4), rotors(3,4), ' 4');


end