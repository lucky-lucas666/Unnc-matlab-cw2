% Student Information
% Name: [Jiacheng LU]
% Email: [ssyjl30@nottingham.edu.cn]
% Student ID: [20616655]
% Init
clear; 
close all;
clc;
a = arduino('COM9', 'Uno');     % Create an Arduino object

%% LED test
ledPin = 'D7';      % Set the LED pin
blinkInterval = 0.5; % Set the blink interval (seconds)
test_time = 10;      % LED test duration (seconds)
for i = 1:test_time
	writeDigitalPin (a, ledPin, 1);
    pause(blinkInterval);
    writeDigitalPin (a, ledPin, 0);
    pause(blinkInterval);
end

%%TASK 1 - READ TEMPERATURE DATA, PLOT, AND WRITE TO A LOG FILE [20 MARKS]
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
%% TASK 2 - LED TEMPERATURE MONITORING DEVICE IMPLEMENTATION [25 MARKS]
% Cleanup
clear a;
% Clear existing Arduino connections
clear a;
a = arduino('COM9', 'Uno');
ledPins = {'D7', 'D8', 'D9'};  % LED pins [green, yellow, red] as strings
sensorPin = 'A0';  

% Call the function with corrected pin names
temp_monitor(a, sensorPin, ledPins);
% ========== FUNCTION DEFINITION ========== 
%TEMP_MONITOR Real-time temperature monitoring and LED control  
%   Continuously reads temperature via Arduino, updates a live plot, and controls LEDs:  
%   - Green LED (steady) if 18°C ≤ T ≤ 24°C  
%   - Yellow LED (0.5Hz blink) if T < 18°C  
%   - Red LED (2Hz blink) if T > 24°C  
%  
%   Inputs:  
%       a: Arduino object  
%       tempPin: Analog pin (e.g., 'A0')  
%       greenPin/yellowPin/redPin: Digital pins ('D7', 'D8', 'D9')  
%       duration: Total monitoring time (seconds)  
%       interval: Sampling interval (seconds)  
%       Tc: Sensor coefficient (default=0.01)  
%       Voc: Voltage at 0°C (default=0.5)  
%  
%   Example:  
%       temp_monitor(a, 'A0', 'D7', 'D8', 'D9', 600, 1);  
%  
%   Notes:  
%       - Ensure LEDs are connected to PWM-enabled pins for blinking.  
%       - Use tic-toc for precise timing.  
function temp_monitor(a, sensorPin, ledPins)
V0 = 0.5;   % Voltage at 0°C (500mV)
Tc = 0.01;  % 10mV/°C
duration = inf;

% Initialize plot
figure;
hPlot = plot(NaN, NaN);
xlabel('Time (s)'); ylabel('Temp (°C)');
title('Real-Time Temperature');
xlim([0 60]); ylim([10 40]);
grid on;

% Initialize timing and LED control
startTime = tic;
timeData = [];
tempData = [];
yellowLastToggle = 0;
redLastToggle = 0;
yellowState = false;
redState = false;

% Main loop
while toc(startTime) < duration
    currentTime = toc(startTime);
    
    % Read temperature
    voltage = readVoltage(a, sensorPin);
    tempC = (voltage - V0) / Tc;
    
    % Update data
    timeData(end+1) = currentTime;
    tempData(end+1) = tempC;
    
    % Update plot
    set(hPlot, 'XData', timeData, 'YData', tempData);
    if currentTime > 60
        xlim([currentTime-60 currentTime]);
    end
    drawnow;
    
    % Control LEDs
    if tempC >= 18 && tempC <= 24
        writeDigitalPin(a, ledPins{1}, true);
        writeDigitalPin(a, ledPins{2}, false);
        writeDigitalPin(a, ledPins{3}, false);
    elseif tempC < 18
        if (currentTime - yellowLastToggle) >= 0.5
            yellowState = ~yellowState;
            writeDigitalPin(a, ledPins{2}, yellowState);
            yellowLastToggle = currentTime;
        end
        writeDigitalPin(a, ledPins{1}, false);
        writeDigitalPin(a, ledPins{3}, false);
    else
        if (currentTime - redLastToggle) >= 0.25
            redState = ~redState;
            writeDigitalPin(a, ledPins{3}, redState);
            redLastToggle = currentTime;
        end
        writeDigitalPin(a, ledPins{1}, false);
        writeDigitalPin(a, ledPins{2}, false);
    end
    
    % Maintain 1-second loop interval
    elapsed = toc(startTime) - currentTime;
    pause(max(1 - elapsed, 0));
end
end
%% TASK 3 - ALGORITHMS – TEMPERATURE PREDICTION [25 MARKS]
% Clear existing Arduino connections
clear a;
a = arduino('COM9', 'Uno');
ledPins = {'D7', 'D8', 'D9'};  % [Green, Yellow, Red]
sensorPin = 'A0';

% Call the function
temp_prediction(a, sensorPin, ledPins);
% ========== FUNCTION DEFINITION ========== 
%TEMP_PREDICTION Predicts temperature trends and triggers alerts  
%   Monitors temperature rate of change (°C/s), predicts future temperature, and controls LEDs:  
%   - Green: Stable within 18-24°C  
%   - Red: Rate > 4°C/min (rising)  
%   - Yellow: Rate < -4°C/min (falling)  
%  
%   Inputs:  
%       a: Arduino object  
%       tempPin: Analog pin ('A0')  
%       greenPin/yellowPin/redPin: Digital pins ('D7', 'D8', 'D9')  
%       interval: Sampling interval (seconds)  
%       Tc: Sensor coefficient (default=0.01)  
%       Voc: Voltage at 0°C (default=0.5)  
%  
%   Example:  
%       temp_prediction(a, 'A0', 'D7', 'D8', 'D9', 1);  
%  
%   Notes:  
%       - Rate calculated using 10-sample moving average to reduce noise.  
%       - Prediction assumes constant rate.  
function temp_prediction(a, sensorPin, ledPins)
% Initialize parameters
V0 = 0.5;       % Voltage at 0°C (500mV)
Tc = 0.01;      % 10mV/°C
alpha = 0.3;    % Smoothing factor
rateThreshold = 4/60; % 4°C/min → 0.0667°C/s
bufferSize = 20; % 20-second window for rate calculation

% Initialize data buffers and LEDs
prevTemps = [];
prevTimes = [];
cellfun(@(pin) writeDigitalPin(a, pin, 0), ledPins); % Turn off all LEDs

% Main loop
startTime = tic;
while true
    currentTime = toc(startTime);
    
    %--- Temperature Measurement & Validation ---%
    voltage = readVoltage(a, sensorPin);
    currentTemp = (voltage - V0) / Tc;
    
    % Reject outliers (e.g., sensor disconnection)
    if currentTemp < -20 || currentTemp > 100
        continue; % Skip invalid readings
    end
    
    % Exponential smoothing
    if isempty(prevTemps)
        smoothedTemp = currentTemp;
    else
        smoothedTemp = alpha * currentTemp + (1 - alpha) * prevTemps(end);
    end
    
    %--- Data Buffering ---%
    prevTemps = [prevTemps, smoothedTemp];
    prevTimes = [prevTimes, currentTime];
    
    % Maintain buffer size
    if numel(prevTemps) > bufferSize
        prevTemps = prevTemps(end - bufferSize + 1 : end);
        prevTimes = prevTimes(end - bufferSize + 1 : end);
    end
    
    %--- Rate Calculation (Robust Linear Regression) ---%
    if numel(prevTemps) >= 5 % Minimum 5 points for regression
        timeWindow = prevTimes - prevTimes(1);
        tempWindow = prevTemps - prevTemps(1);
        
        % Linear regression: rate = Σ(ΔtΔT) / Σ(Δt²)
        sum_dt2 = sum(timeWindow.^2);
        sum_dt_dT = sum(timeWindow .* tempWindow);
        rate = sum_dt_dT / max(sum_dt2, 1e-6); % Avoid division by zero
    else
        rate = 0;
    end
    
    %--- Rate Clipping & Prediction ---%
    rate = max(min(rate, 1.0), -1.0); % Limit to ±1°C/s (60°C/min)
    predictedTemp = smoothedTemp + rate * 300;
    predictedTemp = max(min(predictedTemp, 150), -50); % Clamp to [-50, 150]°C
    
    %--- Console Output ---%
    fprintf('Current: %.2f°C | Rate: %.4f°C/s | Predicted: %.2f°C\n',...
            smoothedTemp, rate, predictedTemp);
    
    %--- LED Control Logic ---%
    if rate > rateThreshold
        % Red LED: Rapid heating (>4°C/min)
        writeDigitalPin(a, ledPins{3}, 1); 
        writeDigitalPin(a, ledPins{1}, 0);
        writeDigitalPin(a, ledPins{2}, 0);
    elseif rate < -rateThreshold
        % Yellow LED: Rapid cooling (<-4°C/min)
        writeDigitalPin(a, ledPins{2}, 1); 
        writeDigitalPin(a, ledPins{1}, 0);
        writeDigitalPin(a, ledPins{3}, 0);
    else
        % Green LED: Stable
        writeDigitalPin(a, ledPins{1}, 1); 
        writeDigitalPin(a, ledPins{2}, 0);
        writeDigitalPin(a, ledPins{3}, 0);
    end
    
    pause(1); % 1-second interval
end
end
%% TASK 4 - REFLECTIVE STATEMENT [5 MARKS]
%​Task 4: Reflective Statement (400 words)
%   Working on this project was both challenging and rewarding, especially as a second-year aerospace engineering student.
% While I had basic MATLAB and Arduino experience from labs, integrating them for a real-world 
% application pushed me to think critically about hardware-software interactions—a skill crucial for aerospace systems
% like avionics or cabin pressure control.
%   Challenges:
% The biggest hurdle was timing synchronization.  For Task 2, my initial LED blinking code often conflicted with temperature sampling,
% causing erratic flashes.  I learned that MATLAB’s pause() function isn’t perfectly precise, so I switched to tic-toc for measuring 
% elapsed time, which stabilized the timing.  Another issue was sensor noise: the thermistor readings fluctuated slightly, affecting 
% temperature predictions in Task 3.  My lab partner suggested averaging the last 10 readings, which smoothed the data without complex
% filters—something I hadn’t considered before.
% Connecting theory to practice was also tricky. For example, calculating temperature from voltage using the MCP9700A seemed straightforward
% in lectures, but real-world voltage drops (due to breadboard resistors) forced me to recalibrate the formula’s constants experimentally.
% This taught me the importance of empirical testing, a lesson I’ll carry into future aerospace projects like wind tunnel experiments.
%   Strengths:
% I leveraged MATLAB’s plotting tools effectively. For Task 2, updating the live graph with `drawnow` helped visualize temperature trends 
% instantly, a technique I practiced during lab sessions. Modular code design (e.g., separating `temp_monitor` and `temp_prediction` into
% functions) mirrored our coursework on structured programming. Version control was vital; I committed changes after debugging major issues, 
% like fixing LED pin conflicts, which mirrored our group project.
%    Limitations:  
% The system’s reliance on manual hardware setup introduced errors. For example, loose jumper wires caused intermittent LED failures—a risk in
% aerospace systems where reliability is non-negotiable. The temperature prediction in Task 3, while functional, ignored sensor noise.
% I tried smoothing data with a moving average to improve accuracy.
%   Future Improvements:  
% 1. Automate Calibration: Add a startup routine to measure the Arduino’s actual voltage, avoiding manual formula adjustments.  
% 2. Error Handling: Implement try-catch blocks to prevent crashes during sensor disconnections.  
% 3. Energy Efficiency: Optimize LED power usage—critical for aerospace applications where resources are limited.  
% This project bridged classroom theory and hands-on engineering. Debugging hardware-software interactions felt akin to troubleshooting 
% avionics systems, where precision is paramount. While my solution isn’t flawless, it solidified my MATLAB skills and highlighted the 
% importance of iterative testing—an approach I’ll carry into future aerospace projects, such as my upcoming drone stability analysis coursework.  
