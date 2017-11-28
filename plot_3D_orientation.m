function plot_3D_orientation(location, euler_angles, armlength)
    %Where location is a column vector, as is the euler_angles
    p = zeros(3,1);
    p = rot_frame_B2I(euler_angles)*...
        [0; 0; armlength];
    figure(1);
    %axes(handles.main_plot); GUI
    hold on
    quiver3(location(1), location(2), location(3), p(1),p(2), p(3)...
    ,'LineWidth',3);
   
end