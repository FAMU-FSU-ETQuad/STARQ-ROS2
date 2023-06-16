%% Square gait

clc
clear
close all

%%

% Leg lengths
L1 = 50;
L2 = 150;

% Gait frequency
f = 0.5; % Hz

% Number of points on trajectory
N = 500;

% Publish rate (for config)
publish_rate = f * N

% Square dimensions
width = 300;
height = 300;
y_offset = 0;

top = [linspace(-width/2, width/2, N/4);
     linspace(height/2 + y_offset, height/2 + y_offset, N/4);
     zeros(1,N/4)];
right = [linspace(width/2, width/2, N/4);
     linspace(height/2 + y_offset, -height/2 + y_offset, N/4);
     zeros(1,N/4)];
bottom = [linspace(width/2, -width/2, N/4);
     linspace(-height/2 + y_offset, -height/2 + y_offset, N/4);
     zeros(1,N/4)];
left = [linspace(-width/2, -width/2, N/4);
     linspace(-height/2 + y_offset, height/2 + y_offset, N/4);
     zeros(1,N/4)];
 
%square = [top, right, bottom, left];
square = [bottom, left, top, right];

figure
plot(square(1,:), square(2,:));

writematrix(square.', '../gait_publisher/gaits/square-test.csv', 'Delimiter', ',')