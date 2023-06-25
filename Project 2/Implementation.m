
clc;
clear all;
close all;
load('laserdata.mat')

%
% 
% Part one GENERATING AN OCCUPANCY GRID MAP
[occu_grid_1] = occupancy_grid_fn (peastSICK, pnorthSICK, 1); 
image(occu_grid_1);
% axis off;
% axis image;
colormap(gray);
title('Occupancy grid with 1m resolution'); 

[occu_grid_0_5] = occupancy_grid_fn (peastSICK, pnorthSICK, 0.5);
figure;
image(occu_grid_0_5);
% axis off;
% axis image;
colormap(gray);
title('Occupancy grid with 0.5m resolution');
% 
% 
[occu_grid_0_2] = occupancy_grid_fn (peastSICK, pnorthSICK, 0.2);
figure;
image(occu_grid_0_2);
% axis off;
% axis image;
colormap(gray);
title('Occupancy grid with 0.2m resolution');
%
% 
[occu_grid_0_1] = occupancy_grid_fn (peastSICK, pnorthSICK, 0.1);
figure; 
image(occu_grid_0_1);
% axis off;
% axis image;
colormap(gray);
title('Occupancy grid with 0.1m resolution');
%
% 
[occu_grid_0_0_5] = occupancy_grid_fn (peastSICK, pnorthSICK, 0.05);
figure;
image(occu_grid_0_0_5);
% axis off;
% axis image;
colormap(gray);
title('Occupancy grid with 0.05m resolution'); 
% 
% 
% 
% Part two GENERATING A GAUSSIAN LIKELIHOOD FIELD FOR THE MAP
for var=1:3
gauss_likeli = Gaussian_likelihood(occu_grid_1./255,var );
figure;
imshow(gauss_likeli);
end
% 
% 
% for var=1:3
% gauss_likeli = Gaussian_likelihood(occu_grid_0_5./255,var );
% figure;
% imshow(gauss_likeli);
% % axis off;
% % axis image;
% % colormap(gray);
% end
%
% 
% for var=1:3
% gauss_likeli = Gaussian_likelihood(occu_grid_0_2./255,1 );
% figure;
% imshow(likelihood);
% % axis off;
% % axis image;
% % colormap(gray);
% end
% 
% 
% for var=1:3
% gauss_likeli = Gaussian_likelihood(occu_grid_0_1./255,1 );
% figure;
% imshow(likelihood);
% % axis off;
% % axis image;
% % colormap(gray);
% end
% 
% 
% for var=1:3
% gauss_likeli = Gaussian_likelihood(occu_grid_0_0_5./255,1 );
% figure;
% imshow(likelihood);
% % axis off;
% % axis image;
% % colormap(gray);


% DISCUSS THE RESULTS
% For higher resolution we attain sharper map.