close all;
% Load the data from CSV file
Ts = 1/333
%data = readtable('input_response_data.csv'); % Update with your CSV file name

% Extract input (Angle) and output (Time) signals
Angle = tilt12vincshorto{:, 1} % Assuming Angle is in the first column
Time = tilt12vincshorto{:, 2} % Assuming Time is in the second column

% Define the input signal (assuming it's a step input of 12V)
input_signal = 12 * ones(size(Time)); % Assuming the input signal is constant

% Plot input and output signals
figure;
subplot(2,1,2);
plot(Time, input_signal);
xlabel('Time');
ylabel('Voltage (V)');
title('Input Voltage');

subplot(2,1,1);
plot(Time, Angle);
xlabel('Time');
ylabel('Angle');
title('Output Angle');

% Estimate the transfer function using System Identification Toolbox
% Assuming a first-order transfer function
sys = tfest(iddata(Angle, input_signal, Ts), 2);

% Display the estimated transfer function
disp('Estimated Transfer Function:');
disp(sys);

% Plot the step response of the estimated transfer function
figure;
step(sys);
title('Step Response of Estimated Transfer Function');