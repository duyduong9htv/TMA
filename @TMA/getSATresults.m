

function w = getSATresults(w)
   %% curve fit the measured bearing, linearly
   whaleTime = w.whaleTime; 
   whaleBearing = w.trueBearings; 

   if w.piecewisefit == 0 
        P = polyfit(whaleTime, whaleBearing, 1);
        whaleBearingFit = polyval(P,whaleTime); 
   else
        whaleBearingFit = piecewiselinearfit(whaleTime, whaleBearing, 20)'; 
   end


    rcvTime = w.rcvTime; 
    rcvLocs = w.rcvLocs; 

    %% interpolate for receiver positions 
    rcvLocsInterp = getRcvLocsInterp(whaleTime, rcvTime, w.rcvLocs);

     %% 
    alpha = zeros(length(rcvLocsInterp)-1,1);    % Instantaneous tow-ship heading
    beta = zeros(length(rcvLocsInterp)-1,1);
    for ii = 2:length(rcvLocsInterp)
        alpha(ii-1) = (rcvLocsInterp(ii,2)-rcvLocsInterp(ii-1,2))./(rcvLocsInterp(ii,1)-rcvLocsInterp(ii-1,1));
        beta(ii-1) = rcvLocsInterp(ii,2)-rcvLocsInterp(ii,1)*alpha(ii-1);
    end
    theta = atan(alpha);
     %% Calculating ship instantaneous velocity 

    timeVec =w.whaleTime; 


    interPingDistance = zeros(length(rcvLocsInterp)-1,1);
    for ii = 2:length(rcvLocs)
        interPingDistance(ii-1) = ddist(rcvLocsInterp(ii, :), rcvLocsInterp(ii-1, :)); 
    end
    dt = diff(timeVec);
    velocity = interPingDistance./dt;

    % bearing derivative 
    dtheta = diff(whaleBearingFit)*pi/180; %should just use the linear fit, don't overcomplicate the problem
    thetaDerivative = dtheta./dt;

    % Get gamma 
    gamma = abs(whaleBearingFit(2:end)*pi/180-pi/2+atan(alpha));
    gamma(gamma>pi) = gamma(gamma>pi) - pi;

    % Get instantaneous projected range tangential to the receiver track line
    r0SAT1 = velocity.*(sin(pi-gamma)).^2./thetaDerivative.*sign(thetaDerivative);
    rSAT1 = velocity.*sin(pi-gamma)./thetaDerivative.*sign(thetaDerivative);

    %% only valid ranges are included 
    inds = find(r0SAT1(:) > 0); 
    r0SAT1 = r0SAT1(inds); 
    rcvLocsInterp = rcvLocsInterp(inds, :); 
    timeVec = whaleTime(inds); 
    whaleBearingFit = whaleBearingFit(inds); 
    gamma = gamma(inds); 


    %% Estimated source locations 
    xSAT = rcvLocsInterp(1:end,1) + r0SAT1./sin(gamma).*sind(whaleBearingFit(1:end));
    ySAT = rcvLocsInterp(1:end,2) + r0SAT1./sin(gamma).*cosd(whaleBearingFit(1:end));

    %% Get LLS-fitted source locations 
    coefficient = polyfit(timeVec(1:end),xSAT,1);
    xMotionSATLinear = polyval(coefficient,timeVec(1:end));
    coefficient = polyfit(timeVec(1:end),ySAT,1);
    yMotionSATLinear = polyval(coefficient,timeVec);

    xMeanSATLinear = mean(xSAT);
    yMeanSATLinear = mean(ySAT);

%     rExpected.SAT = sqrt((sourceLocationEstimates.xMotionSATLinear-rcvLocsInterp(1:end,1)).^2 ...
%         +(sourceLocationEstimates.yMotionSATLinear-rcvLocsInterp(1:end,2)).^2);

    %% OUTPUT 

    w.xSATinst = xSAT; 
    w.ySATinst = ySAT;
    w.xSATtrack = xMotionSATLinear; 
    w.ySATtrack = yMotionSATLinear; 

end 
