clc;
close all;
clear;

convertToMat();

load('data.mat','Data');

Ts = 0.005;

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