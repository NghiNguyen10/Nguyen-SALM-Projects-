function [ gaussian_likeli_field ] = Gaussian_likelihood( grid,var )
% 
%  
% 

[r,s] = size(grid);

dist_mat = nan(r*s,1);

gaussian_likeli_field = zeros(r,s);

[x,y] = find(grid==0);
k = 1;
tic;
if var==1
    z = 2.25;
elseif var==2
    z = 5.5;
elseif var==3
    z = 9.75;
end;
% 
% 
while(k<=r)
    l = 1;
    while (l<=s)
        dist = sqrt((x-k).^2 + (y-l).^2);
        dist_mat(:,:) = nan;
        dist_min = min(dist);
        gaussian_likeli_field(k,l) = z*gauss_dist(dist_min,var);
        % gauss_dist = (1/(sqrt(2*pi*var^2)))*exp(-(1/2)*(dis_min^2/var^2));
        l = l+1;
    end
    k = k+1;
end
toc;
end

