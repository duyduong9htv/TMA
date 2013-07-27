
function distance = ddist(a, b)
%function distance = ddist(a, b) calculates distnace between two points
%a(x, y) and b(x1, y2) on a plane. 
distance = sqrt((a(1)-b(1))^2 + (a(2) - b(2))^2); 