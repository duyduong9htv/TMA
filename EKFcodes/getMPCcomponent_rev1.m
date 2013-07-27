function [s1,s2,s3,s4] = getMPCcomponent_rev1(y,w,dt)

s1 = dt*y(1)-y(4)*(w(1)*cos(y(3))-w(2)*sin(y(3)));
s2 = 1+dt*y(2)-y(4)*(w(1)*sin(y(3))+w(2)*cos(y(3)));
s3 = y(1)-y(4)*(w(3)*cos(y(3))-w(4)*sin(y(3)));
s4 = y(2)-y(4)*(w(3)*sin(y(3))+w(4)*cos(y(3)));