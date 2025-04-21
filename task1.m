%% TASK 1 - Temperature Data Logging
% Student Information
% Name: [Jiacheng LU]
% Email: [ssyjl30@nottingham.edu.cn]
% Student ID: [20616655]

% Clear existing Arduino connections
clear a;

% Initialize Arduino
a = arduino('COM9', 'Uno'); % Verify COM port matches your system

% Parameters
sensorPin = 'A0';       % Analog pin for thermistor
duration = 600;         % 10 minutes = 600 seconds (as per Task 1b)
samplingInterval = 1;    % 1-second sampling
tempData = zeros(1, duration); % Preallocate array

% MCP9700A sensor parameters
V0 = 0.5;   % Voltage at 0°C (500mV)
Tc = 0.01;  % Temperature coefficient (10mV/°C)

% Data acquisition
startTime = tic;
for t = 1:duration
    voltage = readVoltage(a, sensorPin);
    tempData(t) = (voltage - V0) / Tc;
    
    % Maintain sampling rate
    elapsed = toc(startTime);
    pause(max(samplingInterval - elapsed, 0));
    startTime = tic;
end

% Calculate statistics
minTemp = min(tempData);
maxTemp = max(tempData);
avgTemp = mean(tempData);

% Generate plot (Task 1c)
figure;
plot(1:duration, tempData);
xlabel('Time (seconds)');
ylabel('Temperature (°C)');
title('Cabin Temperature Monitoring');
grid on;

% File and screen output (Task 1d)
try
    fid = fopen('cabin_temperature.txt', 'w');
    
    % Header (exact format)
    current_date = datetime('now', 'Format', 'dd/MM/yyyy');
    fprintf(fid, 'Data logging initiated - %s\n', current_date);
    fprintf(fid, 'Location - Nottingham\n\n');
    
    % Minute-by-minute data (0-10 minutes)
    for minute = 0:10
        index = minute * 60 + 1;
        % Ensure index doesn't exceed array bounds
        if index > duration
            index = duration; % Use last available data point
        end
        fprintf(fid, 'Minute\t%d\nTemperature\t%.2f C\n', ...
            minute, tempData(index));
    end
    
    % Statistics section (matches Table 1)
    fprintf(fid, '\nMax temp\t%.2f C\n', maxTemp);
    fprintf(fid, 'Min temp\t%.2f C\n', minTemp);
    fprintf(fid, 'Average temp\t%.2f C\n', avgTemp);
    fprintf(fid, '\nData logging terminated');
    
    fclose(fid);
    disp('Data saved successfully to cabin_temperature.txt');
    
    % Display identical output in command window
    outputStr = fileread('cabin_temperature.txt');
    disp(outputStr);
    
catch ME
    error('File write failed: %s', ME.message);
end

% Cleanup
clear a;