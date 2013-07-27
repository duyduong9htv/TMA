for k = 1:length(tata)
    figure(100); 
    hold on; 
    callID = tata(k).whaleID; 
    temp = find(type(:, 1) == callID); 
    k1 = type(temp, 2); 
    if k1 == 1
        c1 = [0 0 1]; %blue for meows 
    elseif k1 == 2
        c1 = [0 0 0] ;%green for feeding
    elseif k1 == 3
        c1 = [0 1 0 ]; %black for bowl-shaped
    elseif k1 == 4
        c1 = [1 0 0]; % red for songs
    else
        c1 = [0.5 0.5 0.5]; %grey-others 
    end 
    plot(whaleTime(k), whaleBearing(k), '*', 'color', c1); 
end
