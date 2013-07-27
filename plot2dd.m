function h = plot2dd(x, varargin)
%function plot2dd(x, varargin) 
%takes in the Nx2 data x and plot. Save typing time. 
% Eg. x = rand(10, 2); 
% plot2dd(x, '*', 'color', 'r', 'markersize', 5); 
h = plot(x(:, 1), x(:, 2), varargin{:}); 
