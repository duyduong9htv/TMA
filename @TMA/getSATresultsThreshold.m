
function w = getSATresultsThreshold(w)
    disp(['Getting SAT results using aperture L = ' num2str(w.L) ' m...'])
    pause(2); 
    rcvLocs = w.rcvLocs; 
    trueBearing = w.trueBearings; 

    if w.piecewisefit ==0
        P1 = polyfit(w.whaleTime, w.trueBearings, 1); 
        trueBearing = polyval(P1, w.whaleTime); 
    elseif w.piecewisefit == 1
        trueBearing = piecewiselinearfit(w.whaleTime, w.trueBearings, w.piecewisewindow)'; 
    end


    L = w.L; 

    N = length(trueBearing); 

    l1 = 0; 
    k = N; 
    while l1 < L
        k = k-1; 
        l1 = ddist(rcvLocs(N, :), rcvLocs(k, :));     
    end

    M = k; 

    locs = []; 
    for ii = 1:M
        l1 = 0; 
        jj = ii; 
        while l1 < L
            jj = jj + 1; 
            l1 = ddist(rcvLocs(jj, :), rcvLocs(ii, :)); 
        end

        theta1 = trueBearing(ii); 
        theta2 = trueBearing(jj); 

        A1 = tand(90 - theta1); 
        A2 = tand(90 - theta2); 

        x11 = rcvLocs(ii, 1); y11 = rcvLocs(ii, 2); 
        x21 = rcvLocs(jj, 1); y21 = rcvLocs(jj, 2); 

        x = (y11 - y21 + A2*x21 - A1*x11)/(A2 - A1); 
        y = A2*(x - x21) + y21; 

        locs = [locs; x y];

    end
    w.xSATinst = locs(:, 1); 
    w.ySATinst = locs(:, 2); 
    disp('Done!'); 

end

        