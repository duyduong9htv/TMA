function [xselect, yselect, inds] = selectPolygon(x, y, xname, yname)
% function [xselect, yselect, inds] = selectPolygon(x, y, xname, yname)
% selects the points that lie within a polygon drawn by the user on a
% figure. 
% INPUTS
%   -x : x variable (Nx1)
%   -y : y variable (Nx1)
%   -xname: string for labeling x axis. 
%   -yname: string for labeling y axis. 
%OUTPUTS: 
%   -xselect: selected x points
%   -yselect: selected y points
%Example: 
% x = linspace(0, 1, 100); y = rand(1, 100); 
%[xselect, yselect, inds] = selectPolygon(x, y, 'Time', 'position')

figure; 
plot(x, y, '*'); 
setFigureAuto; setFont(20); 
xlabel(xname); ylabel(yname); 
disp('Press any key to continue'); 
pause; 


[xPolygon,yPolygon]=getline(gcf,'closed')
in = find(inpolygon(x, y, xPolygon, yPolygon))
hold on; 
plot(x(in), y(in), 'ro'); 
xselect = x(in); 
yselect = y(in); 

inds = in; 
end 