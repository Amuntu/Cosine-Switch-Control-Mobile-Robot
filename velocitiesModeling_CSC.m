close all; clc; clearvars;

b = 0.5; % Base line width.
r = 0.2; % Wheel's Radius.
%{ 
%Dr.essa alghannam's points in the lecture:
T = 30;
map = [[0  5 ]   ; % x-coordinates.
       [1  0 ]   ; % y-coordinates.
       [0  pi/4]]; % theta-orentations.
%}
%{
% Mechatronics applications 2 exam's Points:
T = 50;
map = [ 100 60 10;
        65  15 100;
        0   0   0 ]/10;
%}
%Er. Baher Kherbek's Points in his lecture:
T = 9;
map = [[2  6 ]   ; % x-coordinates.
       [3  2 ]   ; % y-coordinates.
       [pi/4 0 ]]; % theta-orentations.
%{
% My Testing Points :) 
T = 40; % Total simulastion time.
th = linspace(0,2*pi,5);
xt = 5*cos(th) + 5;
yt = 5*sin(th) + 5;
map = [xt;yt;th];
%}

pointsText = cell(1,size(map,2));
pointsText{1} = ['P' num2str(1)];
pointsText{end} = ['P' num2str(size(map,2))];
%%
% This loop is assential to calculate the angles between points (if points
% are greater that 2 Points:
for i = 2:size(map, 2)-1
    pointsText{i} = ['P' num2str(i)];
    map(3,i) = atan2((map(2,i+1)-map(2,i)),...
                     (map(1,i+1)-map(1,i))) ;
end
%% getting constants:
[~, ~, c, n, w] = getCosineSwitchControl_constants(map, T);

dt = 0.1;
t = 0:dt:T;

%Control vectors
v1 = zeros(1,length(t));
v2 = v1;

% Coordinates
X = [map(1,1) v1];
Y = [map(2,1) v1];
Q = [map(3,1) v1];

% Velocities
Q_dot = [0 0 v1]; 
Y_dot = Q_dot; 
X_dot = Q_dot;
wr = v1; 
wl = v1;

j = 1;
odd = 1;
even = 0;
%% Calculate velocities of the Robot over time:
v2int = tan(Q(1));
for i=1:length(t)
    if t(i) > (j*T/n) 
        odd = ~odd; even = ~even;
        j = j + 1;
    end
    v1(i) = c(j)*(1-cos(w*t(i))) * even;
    v2(i) = c(j)*(1-cos(w*t(i))) * odd;
v2int = v2int + v2(i)*dt;
    X_dot(i+1) = v1(i);        % <==> Z1_dot = v1
    Q_dot(i+1) = atan2(v2(i),1);        % <==> Z2_dot = v2
    Q(i+1) = Q(i) + (Q_dot(i+1) * dt);

    Y_dot(i+1) = v1(i) * v2int; % <==> Z3_dot = v1*Z2 ; Z2 = integration of Z2_dot
    
    X(i+1) = X(i) + (X_dot(i+1) * dt);
    Y(i+1) = Y(i) + (Y_dot(i+1) * dt);

    V = norm([X_dot(i) Y_dot(i)]);
    W = Q_dot(i);
    wr(i) = (V + (W*b/2))/r;
    wl(i) = (V - (W*b/2))/r;
end

%% Simulate moving robot:
figure
pause(2)
for i = 1:length(X)-1
    plot(X(1:i),Y(1:i),'r','LineWidth',2)
    hold on
    plot(map(1,:), map(2,:),'ok')
    text(map(1,:), map(2,:),pointsText)
    plot(min(X)-2*b,min(Y)-2*b,max(X)+2*b,max(Y)+2*b)
    plot_robot(X(i),Y(i) , Q(i), b*1.3, b, r, r/2)
    hold off
    axis equal
    pause(dt/10)
end

%% plotting resultes:
figure % plot Control vectors
grid on
title('Control')
xlabel('time')
plot(t,v1,'b',t,v2,'g--')
legend('v_1','v_2')

figure  % plot change of position over time
hold on; grid on
title('Position over Time')
xlabel('time')
plot(t,X(2:end),'b')
plot(t,Y(2:end),'g--')
plot(t,Q(2:end),'r-.')
legend('X','Y','Q')

figure  % plot change of Robot velocities over time
hold on; grid on
title('Velocities over Time')
xlabel('time')
plot(t,X_dot(2:end-1),'b')
plot(t,Y_dot(2:end-1),'g--')
plot(t,Q_dot(2:end-1),'r-.')
legend('X_d_o_t','Y_d_o_t','Q_d_o_t')

figure  % plot Speeds of wheels over time
hold on; grid on
title('wheels Speeds over Time')
xlabel('time')
plot(t,wl,'Color', [0 .5 .5])
plot(t,wr,'--','Color', [.5 0 .5])
legend('W_l','W_r')