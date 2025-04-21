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