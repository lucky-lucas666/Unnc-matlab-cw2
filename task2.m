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
