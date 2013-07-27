function x = stateVectorLPC2Cartesian_rev1(y)

x = zeros(4,length(y));
x(1,:) = exp(y(4,:)).*sin(y(3,:));
x(2,:) = exp(y(4,:)).*cos(y(3,:));
x(3,:) = exp(y(4,:)).*(y(2,:).*sin(y(3,:)) + y(1,:).*cos(y(3,:)));
x(4,:) = exp(y(4,:)).*(y(2,:).*cos(y(3,:)) - y(1,:).*sin(y(3,:)));