%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Written by: Reynaldo Villarreal Zambrano, William Meacham, Josh Fleck
% Date: 5.4.23
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [crossing_time] = moving_train(design_parameters)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The method 'moving_train' takes an array of design parameters as input 
% and returns the time it takes for a moving train to cross a distance of 
% 10 meters. The function uses the fourth-order Runge-Kutta method to solve a
% system of ordinary differential equations that describe the motion 
% of the train.
%
% Input Arguments:
% design_parameters - An array that contains values for
%                     Lt = design_parameters(1) - length of train [m]
%                     ro = design_parameters(2) - radius of train [m]
%                     P0gauge = design_parameters(3) - inital tank gauge pressure [Pa]
%                     rg = design_parameters(4) - pinion gear radius [m]
%                     Lr = design_parameters(5) - length of piston stroke [m]
%                     rp = design_parameters(6) - radius of piston [m]
%                     pt = design_parameters(7) - density of material
%
% Output Arguments:
% crossing_time - Time at which the train crosses the finish line (10 m)
%                 If value = 0 -> never assigned a crossing time
%                 If value < 100 -> train operates
%                 If value = 100 -> invalid combination and/or never runs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
crossing_time = 0;
% step 1
Lt = design_parameters(1); % length of train [m]
ro = design_parameters(2); % radius of train [m]
P0gauge = design_parameters(3); % inital tank gauge pressure [Pa]
rg = design_parameters(4); % pinion gear radius [m]
Lr = design_parameters(5); % length of piston stroke [m]
rp = design_parameters(6); % radius of piston [m]
pt = design_parameters(7);  % density of material
% step 2
% check design constraints to ensure that they are allowed in our system
% if any design values aren't valid, dont run rk4
% if a design value isnt valid, set crossing_time to a high value (to
% represent its not good) that way code can continue even when given bad
% values that dont actually work
if Lt > 1.5
    crossing_time = 100;
end

% radius of tank
if ro > .095
    crossing_time = 100;
end

% initial tank guage pressure
if P0gauge < 70000 || P0gauge > 200000
    crossing_time = 100;
end

% pinion gear radius
if rg > .0199
    crossing_time = 100;
end

% length of pistion stroke
if Lr < 0.1 || Lr > 0.5
    crossing_time = 100;
end

% radius of pistion
if rp < 0.01 || rp > 0.04
    crossing_time = 100;
end

% step 3
% run the code and get output for velocity and position vs time




% RK-4 for higher order and system of ODE's
% y' = V
% V' = g - pC_DAV^2/(2m)

% Declare constants
h = .01;
tspan = (0:h:15);

% physical parameters
rw = .02; % radius of train wheel [m]
wheel_mass = 0.1;   % kg

r_in = ro/1.15;
volume_outer = pi * ro^2 * Lt;  % volume of outer side of tank
volume_inner = pi * r_in^2 * Lt;    % volume of inner side of tank
total_volume = volume_outer - volume_inner;
train_mass = total_volume * pt;    % train mass


% design parameters
air_density = 1.0;  % kg/m^3
coefficient_static_friction = .7;
drag_coefficient = 0.8;
rolling_resistance_coefficient = 0.03;
atmospheric_pressure = 101325;  % Pa
g = 9.8;    % [m/s^2]

x0 = 0; % initial condition    
v0 = 0; % initial condition

% declare parameters array
parameters = [Lr, rg, rp, Lt, ro, rw, wheel_mass, train_mass, P0gauge, air_density, coefficient_static_friction, drag_coefficient, rolling_resistance_coefficient, atmospheric_pressure, pt, g];

% Declare initial conditions array
y = [x0, v0];
odefunc = @(t, y) train_motion(t, y, parameters);

% Run RK4
if crossing_time == 0
    % call rk4
    [~, yValuesRK4] = rk4(odefunc, y, tspan, h);
    % run through all values of position
    for i = 1:length(yValuesRK4(:, 1))
        xValue = yValuesRK4(i, 1);
        % if theres a value that crossed the finish line
        if xValue >= 10
            timeIndex = i;  % time index
            crossing_time = tspan(timeIndex); % time that train crosses finish line
            break
        end
    end
    % if we go over the end of the track
    if yValuesRK4(end, 1) > 12.5
        crossing_time = 100; % arbitrarily high crossing time
    end
end
% if no crossing time value was assigned or remained at 100 (not valid
% solution)
if crossing_time == 100
    crossing_time = 100; % redundant but more so for clarity
end
% if time remained at 0 (no assigned crossing time)
if crossing_time == 0
    crossing_time = 100; % arbitrarily high crossing time
end
end % function 
