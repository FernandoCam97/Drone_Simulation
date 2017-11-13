function plot_3D_orientation(location, euler_angles, armlength)
    %Where location is a column vector, as is the euler_angles
    Positions = zeros(3,2);
    Positions = [location, rotation_matrix(euler_angles)*...
        [0; 0; armlength/2] + location];
    figure(1)
    hold on
    quiver3(location(1), location(2), location(3), Positions(1,2),...
        Positions(2,2), Positions(3,2),'LineWidth',3);
   
end