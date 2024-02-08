%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Driver Script
% Team: Reynaldo Villarreal Zambrano, William Meacham, Josh Fleck
% Date: 5.4.23
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The driver script 'train_optimization' runs a plethora of code in order
% to simulate the position and velocity of a train using found parameters.
% These parameters are found using a brute force method and then a built-in
% optimization method that comes in matlab. Then, once a combination of
% parameters has been optimized (gotten the best time), we graph both the
% results of an optimal train, as well as the results of a train if we were
% to build it in the real world.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear, clc, close all
format long

% define the range of values found in table 2
tic % start timer 
Nums = 6;   % number of parameters
Lt = linspace(.1, 1.5, Nums);   % train length values
ro = linspace(.03, .095, Nums); % radius of train values  
P0gauge = linspace(70000, 200000, Nums);    % intial tank pressure values
rg = linspace(.002, .0195, Nums);   % radius of pinion gear values
Lr = linspace(.1, .5, Nums);    % length of piston stroke values
rp = linspace(.02, .04, Nums);  % radius of pinion gear values
Lb = [.1, .03, 7000, .002, .1, .02];    % Lower bounds for all parameters
Ub = [.3, .095, 200000, .0195, .5, .04];    % upper bounds for all parameters

% create a function handle of moving_train and thatll be fed into the
% optimization routine (using brute force)
fun = @(design_parameters) moving_train(design_parameters);

% x0 is an array of pretty good values found using brute force (initial
% guess)
% x is the parameters, fval is the time found using those optimized
% parameters
%v[x, fval] = fmincon(fun, x0, [], [], [], [], lower_bound, upper_bound)

bestTimeVal = 100; % set to some arbitrary, large initial value
% iterates over every parameter value guess in order to find the best
% combination. O behavior is O(number of guesses^(number of parameters))
numberIterations = 0;

% Nested for loops that runs through all the possible combinations of
% parameters and finds which combination gives us the lowest running time
% and also ensures that it actually crosses the finish line without going
% over the end of the track
for i = 1:length(Lt)
    for j = 1:length(ro)
        for k = 1:length(P0gauge)
            for l = 1:length(rg)
                for m = 1:length(Lr)
                    for n = 1:length(rp)
                        % Declare the current designParameters of the
                        % current iteration
                        designParams = [Lt(i), ro(j), P0gauge(k), rg(l), Lr(m), rp(n), 1400];
                        % find the crossingTime of this current set of
                        % design parameters
                        [crossingTime] = moving_train(designParams);
                        % increase the number of iteration
                        numberIterations = numberIterations + 1;
                        % if the calculated time is better than the best
                        % current time, replace it and save the combination
                        % of parameters
                        if(crossingTime < bestTimeVal)
                            bestTimeVal = crossingTime;
                            bestParams = designParams;
                        end
                    end
                end
            end
        end
    end
end

% x0 is an array of pretty good values found using brute force (initial
% guess)
% x is the parameters, fval is the time found using those optimized
% parameters

% use fminsearch to finalize the optimization of our parameters
[parameters,fval] = fminsearch(fun, bestParams);

% optimize the density value with our current best parameters
densityValues = [1400, 1200, 7700, 8000, 4500, 8940, 2700];
bestTime = 100;
% loop to run through all the possible densities
for i = 1:length(densityValues)
    % get current density
    pt = densityValues(i);
    iterParameters = parameters;
    % declare new parameters array using our density in this current
    % iteration
    iterParameters(end) = pt;
    % find the run time of this combination
    currentTime = moving_train(iterParameters);
    % if the currentTime is less than last bestTime, replace it and save
    % the best density
    if currentTime < bestTime
        bestTime = currentTime;
        bestDensity = pt;
        parameters = iterParameters;
    end
end
computationalTime = toc;    % stop the timer
fprintf('The time it takes to run through our optimization algorithm is %.3f', computationalTime)

% plot optimal parameters by calling moving_train_graph that returns
% yValuesRK4 (position and velocity), tspan, and the time that it takes to
% cross the finish line
[yValuesRK4, tspan, crossingTime] = moving_train_graph(parameters);
position = yValuesRK4(:, 1);
figure
hold on
% Create plot with left vertical axis for position
yyaxis left
plot(tspan, position, 'b')
ylabel('Position [m]','Color', 'b')
yline(10, 'k')  % finish line
yline(12.5, 'k')    % end of track
% plot velocity vs time
velocity = yValuesRK4(:, 2);
yyaxis right
plot(tspan, velocity, 'r')
ylabel('Velocity [m/s]', 'Color', 'r');
title('Position vs Time (optimal parameters)')
xlabel('Time [s]')
xline(crossingTime, 'g') % plot crossing time 
labels = {sprintf('Position = %.3f meters', position(end, 1)), 'Finish Line = 10 meters', 'End of track = 12.5 meters', 'Velocity [m/s]', sprintf('Crossing Time = %.3f seconds', crossingTime)};
legend(labels, 'Location', 'southeast');

%% real world values and simulation
% declare real world values (from found parts)
Lt = 1.5;
ro = 0.0762;
P0gauge = 100350;
rg = .0056;
Lr = .3048;
rp = .01905;
pt = 1400;
% declare parameters array of our real values
RealParameters = [Lt, ro, P0gauge, rg, Lr, rp, pt];

% plot optimal parameters
[yValuesRK4, tspan, RealCrossingTime] = moving_train_graph(RealParameters);
position = yValuesRK4(:, 1);
figure
hold on
% Create plot with left vertical axis for position
yyaxis left
plot(tspan, position, 'b')
ylabel('Position [m]','Color', 'b')
yline(10, 'k') % plot finish line
yline(12.5, 'k') % plot end of track
% plot velocity vs time
velocity = yValuesRK4(:, 2);
yyaxis right
plot(tspan, velocity, 'r')
ylabel('Velocity [m/s]', 'Color', 'r');
title('Position vs Time (real parameters)')
xlabel('Time [s]')
xline(RealCrossingTime, 'g') % plot crossing time 
labels = {sprintf('Position = %.3f meters', position(end, 1)), 'Finish Line = 10 meters', 'End of track = 12.5 meters', 'Velocity [m/s]', sprintf('Crossing Time = %.3f seconds', RealCrossingTime)};
legend(labels, 'Location', 'southeast');

% end of driver script
