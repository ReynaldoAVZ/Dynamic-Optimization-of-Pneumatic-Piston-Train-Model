%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Written by: Reynaldo Villarreal Zambrano
% Date: 4.16.23
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [t, y] = rk4(odefun,y0, tspan, h)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The method RK_4 uses the Runge-Kutta 4th order method in order to
% approximate an ODE given the initial conditions of the system. 
%
% Runge-Kutta Method: y_i+1 = y_i + 1/6(k_1 + 2k_2 + 2k_3 + k4) * h
% where:
% k1 = f(x_i, y_i)
% k2 = f(x_i + 1/2 * h, y_i + 1/2 * k1 * h)
% k3 = f(x_i + 1/2 * h, y_i + 1/2 * k2 * h)
% k4 = f(x_i + h, y_i + k3 * h)
% 
% Input Parameters:
% odefun - function handle that contains ODE's
% y0 - vector of initial conditions
% tspan - Length of time that system runs.
% h - step-size used in method
%
% Output Paramaters:
% y - vector of dependent variables at x = x_n
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Get total number of iterations
N = length(tspan);  % Number of iterations that Runge-Kutta method will run

% Declare y vector that contains results
y = zeros(N, length(y0));

% Place initial conditions into y vector
y(1, :) = y0(1, :);

% Runge-Kutta method
for i = 1:N-1
    k1 = odefun(tspan(i), y(i, :)); % assign [dydt, dvdt]
    k2 = odefun(tspan(i) + (h / 2), y(i, :) + (h / 2) * k1);
    k3 = odefun(tspan(i) + (h / 2), y(i, :) + (h / 2) * k2);
    k4 = odefun(tspan(i) + h, y(i, :) + h * k3);
    for j = 1:length(y0)
        y(i + 1, j) = y(i, j) + (h / 6) * (k1(1, j) + 2 * k2(1, j) + 2 * k3(1, j) + k4(1, j));
    end
end
t = tspan;
end % function
