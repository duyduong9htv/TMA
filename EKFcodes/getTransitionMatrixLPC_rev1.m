function F = getTransitionMatrixLPC_rev1(s1,s2,s3,s4,w,dt,y)

c1 = s1/(s1^2+s2^2);
c2 = s2/(s1^2+s2^2);
c3 = (s3*(s1^2-s2^2)+2*s1*s2*s4)/(s1^2+s2^2)^2;
c4 = (s4*(s1^2-s2^2)-2*s1*s2*s3)/(s1^2+s2^2)^2;

e1 = w(1)*cos(y(3))-w(2)*sin(y(3));
e2 = w(3)*sin(y(3))+w(4)*cos(y(3));
e3 = w(1)*sin(y(3))+w(2)*cos(y(3));
e4 = w(3)*cos(y(3))-w(4)*sin(y(3));

h = exp(-y(4));

F(1,1) = c2+c4*dt;
F(1,2) = -c1+c3*dt;
F(1,3) = (c1*e4+c2*e2+c4*e3-c3*e1)*h;
F(1,4) = (-c1*e2+c2*e4+c3*e3+c4*e1)*h;
F(2,1) = c1-c3*dt;
F(2,2) = c2+c4*dt;
F(2,3) = (c1*e2-c2*e4-c3*e3-c4*e1)*h;
F(2,4) = (c1*e4+c2*e2-c3*e1+c4*e3)*h;
F(3,1) = c2*dt;
F(3,2) = -c1*dt;
F(3,3) = 1+(c1*e1+c2*e3)*h;
F(3,4) = (-c1*e3+c2*e1)*h;
F(4,1) = c1*dt;
F(4,2) = c2*dt;
F(4,3) = (c1*e3-c2*e1)*h;
F(4,4) = 1+(c1*e1+c2*e3)*h;



