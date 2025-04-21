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
