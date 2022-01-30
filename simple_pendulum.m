function [Theta_mat,Theta_dot_mat,Theta_double_dot_mat] = simple_pendulum(Theta_o,Theta_dot_o)
%==========================================================================
% Simple Pendulum
%--------------------------------------------------------------------------
% This function solves the dynamic equation of a simple pendulum based on 
% initial conditions given by the user. The initial condition include 
% intial angular position and initial angular velocity.
%==========================================================================

%==========================================================================
% Default Arguments
% The following piece of code gives default values to the arguments
%==========================================================================
if(nargin == 1)
    Theta_dot_o = 0;
elseif (nargin == 0)
    Theta_o = pi/3;
    Theta_dot_o = 0;
end

%==========================================================================
% Pendulum Parameters
%--------------------------------------------------------------------------
% The following are the parameters of the simple pendulum
%==========================================================================
m = 0.5; % Mass (kg)
g = 9.81; % Gravitational Acceleration (m/s2)
l = 0.5; % Length of the pendulum (m)
b = 0.15; % damping coefficient (kg.m/s)
T = 10.0; % Total Time (s)
tspan = [0 T]; % Time span

%==========================================================================
% Initial State vector
%==========================================================================
x_o = [Theta_o; Theta_dot_o];

%==========================================================================
% Solving the Simple pendulum Differential equation
%==========================================================================
[t, x_mat] = ode45(@(t,x)SP_ode(t,x,g,m,l,b),tspan,x_o);

%==========================================================================
% Time matrix, Delta time matrix and Total number of steps for solution
% These are the values used for the plotting
%==========================================================================
t_mat = t;
t_mat_1 = t_mat(1:size(t_mat)-1);
t_mat_2 = t_mat(2:size(t_mat));
delta_t = t_mat_2 - t_mat_1;
delta_t = [delta_t;0.01];
Steps = size(t_mat);

%==========================================================================
% Angular position, angular velocity and angular acceleration Matrices
% These are used for plotting
%==========================================================================
Theta_mat = x_mat(:,1);
Theta_dot_mat = x_mat(:,2);
Theta_double_dot_mat = -(g/l)*sin(x_mat(:,1)) - (b/(m*l*l))*x_mat(:,2);

%==========================================================================
% Pendulum Bob Plot
%--------------------------------------------------------------------------
% Plot function for a circle with desired Radius and Centre
%==========================================================================
cplot = @(Name,r,x0,y0) plot(Name,x0 + r*cos(linspace(0,2*pi)),y0 + ...
    r*sin(linspace(0,2*pi)),'-');

%==========================================================================
% Pendulum simulation and Graph plots
%==========================================================================
for i=1:Steps
    tiledlayout(2,2);

    % Simple Pendulum simulation
    SP_sim = nexttile;
    plot(SP_sim,[-0.1 0.1],[0 0]);
    hold on
    plot(SP_sim,[0 l*sin(Theta_mat(i))],[0 -l*cos(Theta_mat(i))]);
    cplot(SP_sim,0.04,l*sin(Theta_mat(i)),-l*cos(Theta_mat(i)));
    title('Simple Pendulum Simulation');
    xlabel('X (m)');
    ylabel('Y (m)');
    axis([-1 1 -1 1]);
    pbaspect([1 1 1]);
    
    % Theta Plot
    Theta_plot = nexttile;
    plot(Theta_plot,t,Theta_mat);
    hold on
    title('Angular Position');
    xlabel('Time (s)');
    ylabel('Angular Position (rad)');
    plot(Theta_plot,[t(i) t(i)],[min(Theta_mat) max(Theta_mat)],'-.');
    
    % Theta dot Plot
    Theta_dot_plot = nexttile;
    plot(Theta_dot_plot,t,Theta_dot_mat);
    hold on
    title('Angular Velocity');
    xlabel('Time (s)');
    ylabel('Angular Velocity (rad/s)');
    plot(Theta_dot_plot,[t(i) t(i)], ...
        [min(Theta_dot_mat) max(Theta_dot_mat)],'-.');

    % Theta double dot Plot
    Theta_double_dot_plot = nexttile;
    plot(Theta_double_dot_plot,t,Theta_double_dot_mat);
    hold on
    title('Angular Acceleration');
    xlabel('Time (s)');
    ylabel('Angular Accleration (rad/s^2)');
    plot(Theta_double_dot_plot,[t(i) t(i)], ...
        [min(Theta_double_dot_mat) max(Theta_double_dot_mat)],'-.');

    % Pause
    pause(delta_t(i));
    hold off
end
end