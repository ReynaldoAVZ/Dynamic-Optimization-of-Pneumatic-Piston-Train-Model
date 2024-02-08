%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Written by: Reynaldo Villarreal Zambrano
% Date: 5.4.23
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function results = train_motion(~, y, params)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% train_motion Ordinary differential equation function that models the
%   dynamics of a falling ball.
%
% dydt = train_motion(t, y, params) calculates the value of the derivative of the
% velocity at the current time t and velocity y as well as the position x.
%
% INPUT Arguments:
%       t - Current time
%       y - state of position and velocity of moving train (vector)
%       params - Physical system parameters
%       Lr = length of piston stroke [m]
%       rg = pinion gear radius [m]
%       rp = radius of piston [m]
%       Lt = length of train [m]
%       ro = radius of tank [m]
%       rw = radius of train wheel [m]
%       wheel_mass =  [kg]
%       train_mass = kg
%       P0gauge = inital tank gauge pressure [Pa]
%       air_density = kg/m^3
%       coefficient_static_friction = unitless
%       drag_coefficient = unitless
%       rolling_resistance_coefficient = unitless
%       atmospheric_pressure = Pa
%       pt = density of train material [kg/m^3]
%       g = 9.8;    % [m/s^2]
%
% OUTPUT Arguments: 
% results - vector containing results of evaluating the n first-order
%           ODE at time t. First entry is train velocity and second is 
%           the acceleration
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Lr = params(1);   % length of piston stroke [m]
rg = params(2);  % pinion gear radius [m]
rp = params(3);  %   radius of piston [m]
Lt = params(4);   % length of train [m]
ro = params(5);  % radius of tank [m]
rw = params(6); % radius of train wheel [m]
wheel_mass = params(7);   % [kg]
train_mass = params(8);    % kg

% design parameters
P0gauge = params(9);    % inital tank gauge pressure [Pa]
air_density = params(10);  % kg/m^3
coefficient_static_friction = params(11);
drag_coefficient = params(12);
rolling_resistance_coefficient = params(13);
atmospheric_pressure = params(14);  % Pa
g = params(16);    % [m/s^2]

% Assign inputs to variables
x = y(1, 1);  % position
v = y(1, 2);  % velocity

% calculate area of train with given paramaters and piston
A = pi * (ro/1.15)^2;    % area of the train    
Ap = pi * (rp)^2;  % area of the piston
% calculate Volume

Volume = A * Lt; % volume of the tank with pressure

% pressure
P = P0gauge + atmospheric_pressure;

% Calculate derivative of position
dydt = v;

% Find maximum distance of acceleration phase
L_alpha = (Lr * rw) / rg;

% calculate the forces occuring
Fd = (1/2) * drag_coefficient * air_density * A * (dydt)^2;
Fr = rolling_resistance_coefficient * train_mass * g;
Ft = (Ap * (rg/rw)) * ((P * Volume) / (Volume + (Ap * rg / rw * x)) - atmospheric_pressure);
% determine whether or not train is still accelerating or not
if x >= L_alpha   % if the distance traveled is greater than acceleration phase, deaccelerate
   if v > 0
    dvdt = (1 / train_mass) * (-Fd - Fr);
   else
       dvdt = (1/train_mass) * (Fd + Fr);
   end
else
  % acceleration
    dvdt = (1 / (train_mass + wheel_mass)) * (Ft - Fd - Fr);
      if Ft > (coefficient_static_friction * (train_mass / 2) * g)
          dvdt = 0;
          dydt = 0;
      end
end

% if the velocity is near zero and we're done accelerating, stop moving
if dydt < 0.001 && x >= L_alpha
  dvdt = 0;
  dydt = 0;
end

% Assemble output
results = [dydt, dvdt];

end % function