function [ occu_grid ] = occupancy_grid_fn( east_coord, north_coord, resol )
% The function is to generate the Occupancy Grid map
%
%
% 
[row,col]= size(north_coord);
u_coord = reshape(east_coord, row*col,1); 
v_coord = reshape(north_coord, row*col,1);

occu_grid_dim=[min(min(east_coord)) min(min(north_coord)) 
               max(max(east_coord)) max(max(north_coord))]; 
            
occu_grid_dim = round(occu_grid_dim)/resol;
            
occu_grid = zeros(occu_grid_dim(2,1)- occu_grid_dim(1,1), occu_grid_dim(2,2)- occu_grid_dim(1,2));

u_coord=round((u_coord - min(min(east_coord))+1)/resol);
v_coord=round((v_coord - min(min(north_coord))+1)/resol);

u_coord(isnan(u_coord))= 1;
v_coord(isnan(v_coord))= 1;

k = 1440*399;
tic;
while(k~=0)
occu_grid(u_coord((1440*399)-k+1 ,1),v_coord((1440*399)-k+1,1)) = 255;
k=k-1;
end
toc;
occu_grid=255 - rot90(occu_grid);
% 
% 
% 
end

