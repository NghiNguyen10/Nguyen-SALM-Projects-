%  Project 01 - Test
%
%   
clear all
close all
randn('state',0) % Reset random number generator

load data01

%  Define measurement variance

sigma_r = 100;

% ----------------------------------------------------------------
%  GENERATE THE MEASUREMENTS
% ----------------------------------------------------------------
for k = 1:length(rx),

    % Generate errors
    v = [sigma_r*randn(1,1); sigma_r*randn(1,1)];
    
    % True position and velocity
    x = [rx(k); ry(k)];
    
    % Make measurement vector
    z(:,k) = x + v;
end

% ----------------------------------------------------------------
%  LKF - State Vector: x, x_dot, y, y_dot
% ----------------------------------------------------------------

dt = 1;

%
% INSERT YOU FILTER HERE: use variable x_est for your estimate of the state
%
%Initializing the nessary variables
%sigma_w = 1;
sigma_w = 10;
%sigma_w = 100;

%A = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1]; % A matrix
Phi = [1 dt 0 0; % Changes to A matrix based on Constant velocity model
       0 1 0 0;
       0 0 1 dt;
       0 0 0 1];
 
H = [1 0 0 0;
     0 0 1 0]; % C matrix
 
% u = 0.05;  %Aceleration magnitude
%u=[0 vx(:) 0 vy(:)]; %Action vector based on Constant velocity model

R = [sigma_r 0;
     0 sigma_r]; %Mean and variance of the random noise

Q = (sigma_w^2)*[(dt^4)/4 (dt^3)/2 0 0;
     (dt^3)/2 dt^2 0 0;
      0 0 (dt^4)/4 (dt^3)/2;
      0 0 (dt^3)/2 dt^2]; %Co variance Matrix
%Q = (sigma_w^2)*[(dt^3)/3 (dt^2)/2 0 0; (dt^2)/2 dt 0 0; 0 0 (dt^3)/3 (dt^2)/2; 0 0 (dt^2)/2 dt];
% initialize the initial vectors
x_est(:,1) = [rx(1); 0; ry(1); 0];
P_pre = 1000*eye(4); % Initial Covarience matrix

%Start Kalman Filter
for k=1:length(rx) - 1
    
   %Co_var = A * Co_var * A' + Q; % Predict the next Co variance matrix
   
    K = P_pre *H' * inv(H * P_pre * H' + R); % Calculating the Kalman Gain (require optimization)???
    
    x_est(:,k) = x_est(:,k) + K * ( z(:,k) - H * x_est(:,k)); % Calculate the Actual After Measure ment Update
    
    P_pre = (eye(4) - K*H) * P_pre; %Update the Covariance 
    
   x_est(:,k+1) =  Phi*x_est(:,k);
    P_pre = Phi*P_pre*Phi' + Q; 
end
%
%
figure
plot(z(1,:),z(2,:),'LineWidth', 2)
grid on
axis([-5000 5000 -2000 4000])
axis('square')
ylabel('y position [m]','FontSize',14,'FontWeight','bold')
xlabel('x position [m]','FontSize',14,'FontWeight','bold')
title('True and Estimated Flight Path','FontSize',14,'FontWeight','bold')
 hold on
 plot(x_est(1,:),x_est(3,:),'g','LineWidth', 2)
 hold off


figure
subplot(2,1,1)
plot(vx,'LineWidth', 2);
 hold on
 plot(x_est(2,:),'g','LineWidth', 2);
 hold off
grid on
ylabel('vx estimate [m/s]','FontSize',14,'FontWeight','bold')
title('LKF Velocity Estimates','FontSize',14,'FontWeight','bold')
subplot(2,1,2)
plot(vy,'LineWidth', 2);
 hold on
 plot(x_est(4,:),'g','LineWidth', 2);
 hold off
grid on
ylabel('vy estimate [m/s]','FontSize',14,'FontWeight','bold')
xlabel('run time in seconds','FontSize',14,'FontWeight','bold')