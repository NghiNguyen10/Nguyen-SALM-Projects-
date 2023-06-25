%   Project 01
%
%   LINEAR KALMAN FILTER
%
%   <Akshay S Bharadwaj>
%
%   Due: 3 February 2015
%
clc;
clear all
close all
randn('state',0) % Reset random number generator

load data01

%   Define measurement variance
sigma_r = 25;

beacon1 = [-50000; 50000];
beacon2 = [ 50000; 50000];

% ----------------------------------------------------------------
%  GENERATE THE MEASUREMENTS
% ----------------------------------------------------------------
for k = 1:length(rx),

    % Measurements
    r1 = sqrt( (rx(k)-beacon1(1))^2 + (ry(k)-beacon1(2))^2 ) + sigma_r*randn(1,1);
    r2 = sqrt( (rx(k)-beacon2(1))^2 + (ry(k)-beacon2(2))^2 ) + sigma_r*randn(1,1);
    
    % Make measurement vecor
    z(:,k) = [r1; r2];
end

% ----------------------------------------------------------------
%  LKF - State Vector: x, x_dot, y, y_dot
% ----------------------------------------------------------------

dt = 1;

%
% INSERT YOU FILTER HERE: use variable x_est for your estimate of the state
%
%initializinf all the variables
sigma_w = 1;
%sigma_w = 10;
%sigma_w = 100;

% The Phi matrix for linear combination
Phi = [1 dt 0 0
       0 1 0 0
       0 0 1 dt
       0 0 0 1];
   
%Initial Co_Variance matrix
P = 10000*eye(4);

% Error co-variance matrix
Q = sigma_w *[dt^3/3 dt^2/2 0 0
     dt^2/2 dt 0 0
     0 0 dt^3/3 dt^2/2
     0 0 dt^2/2 dt ];

 %Error in measurement is
R = [sigma_r^2 0 
     0 sigma_r^2 ];
%Measurement update matrix 
% H = [1 0 0 0
%      0 0 1 0];
   
%initial measurement;
x_est=[rx(1,1)
       %vx(1,1)
       0
       ry(1,1)
       %vy(1,1)];
       0];
% do Kalman Filter
for t = 1:length(rx)
    
    z_est(:,t) = [sqrt((beacon1(1) - x_est(1,t))^2 + (beacon1(2) - x_est(3,t))^2)
             sqrt((beacon2(1) - x_est(1,t))^2 + (beacon2(2) - x_est(3,t))^2)];
    
    H=[(x_est(1,t)- beacon1(1))/z_est(1,t) 0 (x_est(3,t)- beacon1(2))/z_est(1,t) 0
       (x_est(1,t)- beacon2(1))/z_est(2,t) 0 (x_est(3,t)- beacon2(2))/z_est(2,t) 0];
         
    %Predecited measuremnet covariance
    K = P*H'/(H*P*H' + R);
    
    x_est(:,t) = x_est(:,t) + K * (z(:,t) - z_est(:,t));
    
    Pk=(eye(4)-K*H)*P;
    
    x_est(:,t+1)= Phi*x_est(:,t);
    
    P= Phi * Pk *Phi' + Q;
    
end
%
%
%
figure
plot(rx,ry,'LineWidth', 2)
grid on
axis([-5000 5000 -2000 4000])
axis('square')
ylabel('y position [m]','FontSize',14,'FontWeight','bold')
xlabel('x position [m]','FontSize',14,'FontWeight','bold')
title('True and Estimated Flight Path','FontSize',14,'FontWeight','bold')
%As commented by Maarten
hold on
plot(x_est(1,:),x_est(3,:),'g','LineWidth', 2)
hold off


figure
subplot(2,1,1)
plot(vx,'LineWidth', 2);
%As commented by Maarten
hold on
plot(x_est(2,:),'g','LineWidth', 2);
hold off

grid on
ylabel('vx estimate [m/s]','FontSize',14,'FontWeight','bold')
title('LKF Velocity Estimates','FontSize',14,'FontWeight','bold')
subplot(2,1,2)
plot(vy,'LineWidth', 2);
%As commented by Maarten
hold on
plot(x_est(4,:),'g','LineWidth', 2);
hold off

grid on
ylabel('vy estimate [m/s]','FontSize',14,'FontWeight','bold')
xlabel('run time in seconds','FontSize',14,'FontWeight','bold')
