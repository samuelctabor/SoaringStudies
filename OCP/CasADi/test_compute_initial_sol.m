close all;

start_pos = [0;0];
start_heading = pi/4;

target = [50;50];

velocity = 10;

radius = 20;
N = 300;
T = 30;

Sol = compute_initial_solution( start_pos, start_heading, target, velocity, radius, N, T );

figure,plot(Sol(:,2), Sol(:,1),'b.-');


th=0:0.1:2*pi;
hold on;
plot(target(1)+radius*sin(th), target(2)+radius*cos(th),'r--');


axis equal; grid on; grid minor;