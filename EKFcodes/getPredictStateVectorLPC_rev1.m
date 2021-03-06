function stateVector = getPredictStateVectorLPC_rev1(s1,s2,s3,s4,y)

stateVector = zeros(4,1);
% stateVector(1) = (s1*s4-s2*s3)/(s3^2+s4^2);
% stateVector(2) = (s1*s3+s2*s3)/(s3^2+s4^2);
% stateVector(3) = y(3) + atan2(s3,s4);
% stateVector(4) = y(4) + log(s3^2+s4^2)/2;

stateVector(1) = (s2*s3-s1*s4)/(s1^2+s2^2);
stateVector(2) = (s1*s3+s2*s4)/(s1^2+s2^2);
stateVector(3) = y(3) + atan2(s1,s2);
stateVector(4) = y(4)+log(s1^2+s2^2)/2;