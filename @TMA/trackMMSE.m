function w = trackMMSE(w)
% function w = trackMMSE(w)
% tracks the movement of a target by using the MMSE algorithms applied to a
% runing time window of length Lt 

tmax = max(w.whaleTime); 
inds = w.whaleTime(:)< tmax - w.Lt*60; 
t = w.whaleTime(inds); 
bearings = w.trueBearings(inds); 

trackingResults = []; 
for k = 1:length(t)
    inds1 = find((w.whaleTime(:) - t(k)- w.Lt*60).*(w.whaleTime(:) - t(k))<=0); 
    rcv = w.rcvLocs(inds1, :); 
    rcvTime = w.rcvTime(inds1); 
    measuredBearings = w.trueBearings(inds1); 
    measurementTime = w.whaleTime(inds1); 
    arrayHeading = w.arrayHeading(inds1);     
    w1 = TMA(rcv, rcvTime, measuredBearings,measurementTime, arrayHeading); 
    if k ==1 
        w1 = w1.getMMSEloc; 
    else
        x1 = trackingResults(end, 1); 
        y1 = trackingResults(end, 2); 
        %a function here restricting the search to within 20 km from
        %previous estimate (to reduce the time) 
        [x_hat, y_hat] = estMMSE(w1.rcvLocs, w1.trueBearings, 50,...
            x1-3e3, x1+3e3, y1-3e3, y1+3e3); 
        disp(k/length(t));
        trackingResults = [trackingResults; x_hat y_hat]; 
    end
    
    trackingResults = [trackingResults; w1.xMMSE w1.yMMSE]; 
    clear w1; 
end

w.trackTimeWindowed = trackingResults; 



end
