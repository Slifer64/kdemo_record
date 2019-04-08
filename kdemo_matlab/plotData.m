clc;
close all;
clear;

filename = 'data/data_22';

convertToMat([filename '.bin']);

load([filename '.mat'],'Data');

Ts = 0.002;
vel_thres = 0.05 * Ts;
Data = trimData(Data, vel_thres);

%% ==========  Plot position  ==============
pos_data = Data.TOOL_POS;
if (~isempty(pos_data))
    n_data = size(pos_data,2);
    Time = (0:(n_data-1))*Ts; 
    figure;
    hold on;
    plot(Time, pos_data(1,:), 'LineWidth',1.5);
    plot(Time, pos_data(2,:), 'LineWidth',1.5);
    plot(Time, pos_data(3,:), 'LineWidth',1.5);
    legend({'$x$', '$y$', '$z$'}, 'interpreter','latex', 'fontsize',15);
    title('Position', 'interpreter','latex', 'fontsize',18);
    hold off;
    
    figure;
    plot3(pos_data(1,:), pos_data(2,:), pos_data(3,:), 'LineWidth',1.5);
    title('$3D$ path', 'interpreter','latex', 'fontsize',18);
    xlabel('$x$ [$m$]', 'interpreter','latex', 'fontsize',16);
    ylabel('$y$ [$m$]', 'interpreter','latex', 'fontsize',16);
    zlabel('$z$ [$m$]', 'interpreter','latex', 'fontsize',16);
end

%% ==========  Plot orientation  ==============
orient_data = Data.TOOL_ORIENT;
if (~isempty(orient_data))
    n_data = size(orient_data,2);
    Time = (0:(n_data-1))*Ts; 
    figure;
    hold on;
    plot(Time, orient_data(1,:), 'LineWidth',1.5);
    plot(Time, orient_data(2,:), 'LineWidth',1.5);
    plot(Time, orient_data(3,:), 'LineWidth',1.5);
    plot(Time, orient_data(4,:), 'LineWidth',1.5);
    legend({'$q_w$', '$q_x$', '$q_y$', '$q_z$'}, 'interpreter','latex', 'fontsize',15);
    title('Orientation', 'interpreter','latex', 'fontsize',18);
    hold off;
end

%% ==========  Plot 3D with orientation  ==============
if (~isempty(pos_data) && ~isempty(orient_data))
    fig = figure;
    ax = axes('Parent',fig);
    n_data = size(pos_data,2);
    plot_3Dpath_with_orientFrames(pos_data(:,1:10:n_data), orient_data(:,1:10:n_data), 'axes',ax, 'title','$3D$ path with orientation', 'xlabel','$x$ [$m$]', 'ylabel','$y$ [$m$]', 'zlabel','$z$ [$m$]', ...
        'LineWidth',3, 'LineColor',[0.45 0.26 0.26], 'Interpreter','latex', 'fontSize',14, 'numberOfFrames',10, ...
        'frameScale',0.1, 'frameLineWidth',2.0, 'animated',true, 'Time',0.002);

%  @param[in] frameScale: The scaling of the orientation frames size (optional, default = 1.0).
%  @param[in] frameLineWidth: The linewidth of the orientation frame axes (optional, default = 1.0).
%  @param[in] frameXAxisColor: The color of the x-axis of the orientation frame in rgb format (optional, default = [1 0 0]).
%  @param[in] frameYAxisColor: The color of the y-axis of the orientation frame in rgb format (optional, default = [0 1 0]).
%  @param[in] frameZAxisColor: The color of the z-axis of the orientation frame in rgb format (optional, default = [0 0 1]).
%  @param[in] animated: If true the path is plotted animated (optional, default = false).
%  @param[in] Time: 1xN matrix with timestamps (used to set the animation speed).

end

%% ==========  Plot force  ==============
force_data = Data.TOOL_FORCE;
if (~isempty(force_data))
    n_data = size(force_data,2);
    Time = (0:(n_data-1))*Ts; 
    figure;
    hold on;
    plot(Time, force_data(1,:), 'LineWidth',1.5);
    plot(Time, force_data(2,:), 'LineWidth',1.5);
    plot(Time, force_data(3,:), 'LineWidth',1.5);
    legend({'$f_x$', '$f_y$', '$f_z$'}, 'interpreter','latex', 'fontsize',15);
    title('Force', 'interpreter','latex', 'fontsize',18);
    hold off;
end

%% ==========  Plot torque  ==============
torque_data = Data.TOOL_TORQUE;
if (~isempty(torque_data))
    n_data = size(torque_data,2);
    Time = (0:(n_data-1))*Ts; 
    figure;
    hold on;
    plot(Time, torque_data(1,:), 'LineWidth',1.5);
    plot(Time, torque_data(2,:), 'LineWidth',1.5);
    plot(Time, torque_data(3,:), 'LineWidth',1.5);
    legend({'$\tau_x$', '$\tau_y$', '$\tau_z$'}, 'interpreter','latex', 'fontsize',15);
    title('Torque', 'interpreter','latex', 'fontsize',18);
    hold off;
end

%% ==========  Plot Joints Position  ==============
jpos_data = Data.JOINT_POS;
if (~isempty(jpos_data))
    n_data = size(jpos_data,2);
    Time = (0:(n_data-1))*Ts; 
    figure;
    hold on;
    legend_labels = [];
    for i=1:size(jpos_data,1)
        legend_labels = [legend_labels {['j' num2str(i)]}];
        plot(Time, jpos_data(i,:), 'LineWidth',1.5); 
    end
    legend(legend_labels, 'interpreter','latex', 'fontsize',15);
    title('Joints Position', 'interpreter','latex', 'fontsize',18);
    hold off;
end

%% ==========  Plot Joints Torque  ==============
jtorque_data = Data.JOINT_TORQUE;
if (~isempty(jtorque_data))
    n_data = size(jtorque_data,2);
    Time = (0:(n_data-1))*Ts; 
    figure;
    hold on;
    legend_labels = [];
    for i=1:size(jtorque_data,1)
        legend_labels = [legend_labels {['j' num2str(i)]}];
        plot(Time, jtorque_data(i,:), 'LineWidth',1.5); 
    end
    legend(legend_labels, 'interpreter','latex', 'fontsize',15);
    title('Joints Torque', 'interpreter','latex', 'fontsize',18);
    hold off;
end