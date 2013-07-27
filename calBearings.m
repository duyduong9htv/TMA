function bearings = calBearings(whaleLocsGuess, rcv)
%function bearings = calBearings(whaleLocsGuess, rcv) 
%gives the bearings (theoretical) when the trajectory of the whale is given
%by a Nx2 vector whaleLocsGuess. 

xx = whaleLocsGuess(:, 1); 
yy = whaleLocsGuess(:, 2); 

epsilon = (1e-12); 
iota = (1 - (sign(xx - rcv(:, 1)).^2))*epsilon; %to avoid division by zero 

angle1 = atand((yy - rcv(:, 2))./(xx - rcv(:, 1) + iota)); 
angle1 = angle1 - 90*(1 - sign(xx - rcv(:, 1) + iota));
bearings = 90 - angle1; 

end 
