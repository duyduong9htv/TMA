
function wo = getEKFresults(wo)
    disp('Getting EKF results ...'); 
    whaleTime = wo.whaleTime; 
    whaleBearing = wo.trueBearings; 
    rcvTime = wo.rcvTime; 
    rcvLocs = wo.rcvLocs; 

    [a, b] = unique(whaleTime); 
    whaleTime = a; 
    whaleBearing = whaleBearing(b); 
    rcvTime = rcvTime(b); 
    rcvLocs = rcvLocs(b, :); 

    %% curve fit the measured bearing, linearly
%     P = polyfit(whaleTime, whaleBearing, 1);
%     whaleBearingFit = polyval(P,whaleTime); 
    
    whaleBearingFit = piecewiselinearfit(whaleTime, whaleBearing, wo.piecewisewindow)'; 
%     
%     whaleBearingFit = whaleBearing; 

    %% interpolate for receiver positions 
    rcvLocsInterp = getRcvLocsInterp(whaleTime, rcvTime, rcvLocs);

    %% 
    alpha = zeros(length(rcvLocsInterp)-1,1);    % Instantaneous tow-ship heading
    beta = zeros(length(rcvLocsInterp)-1,1);
    for ii = 2:length(rcvLocsInterp)
        alpha(ii-1) = (rcvLocsInterp(ii,2)-rcvLocsInterp(ii-1,2))./(rcvLocsInterp(ii,1)-rcvLocsInterp(ii-1,1));
        beta(ii-1) = rcvLocsInterp(ii,2)-rcvLocsInterp(ii,1)*alpha(ii-1);
    end
    theta = atan(alpha);

    %% some adjustments for whales 


    timeVec = whaleTime; 
    interPingDistance = zeros(length(rcvLocsInterp)-1,1);
    for ii = 2:length(rcvLocs)
        interPingDistance(ii-1) = ddist(rcvLocsInterp(ii, :), rcvLocsInterp(ii-1, :)); 
    end
    dt = diff(timeVec);
    velocity = interPingDistance./dt;


    % velocity used for MPC-EKF
    u = [velocity(2:end).*cos(theta(2:end))-velocity(1:end-1).*cos(theta(1:end-1)) velocity(2:end).*sin(theta(2:end))-velocity(1:end-1).*sin(theta(1:end-1))];
    u = [[0 0];u];



    % bearingError = var(track.bearingEstimatesTrueNorth*pi/180-track.bearingTrueNorth*pi/180); 
    %this is unfair, since the true bearing is assumed to be known in order to
    %estimate bearing variance
    bearingError = 1; %assuming bearing error variance is 1

    % bearingErrorNl = var(track.bearingEstimatesTrueNorthNl*pi/180-track.bearingTrueNorthNl*pi/180);

    % bearing derivative 
    dtheta = diff(whaleBearingFit)*pi/180; %should just use the linear fit, don't overcomplicate the problem

    % dtheta = diff(expectedMigration)*pi/180; 
    % dtheta1 = diff(expectedMigration1)*pi/180;
    % dtheta3 = diff(track.bearingTrueNorth)*pi/180;
    thetaDerivative = dtheta./dt;
    % thetaDerivative1 = dtheta1./dt;
    % thetaDerivative3 = dtheta3./dt;

    % Get gamma 
    gamma = abs(whaleBearingFit(2:end)*pi/180-pi/2+atan(alpha));
    gamma(gamma>pi) = gamma(gamma>pi) - pi;
    % gamma1 = abs(whaleBearingFit(2:end)*pi/180-pi/2+atan(alpha));
    % gamma1(gamma1>pi) = gamma1(gamma1>pi) - pi;

    % Get instantaneous projected range tangential to the receiver track line
    r0SAT1 = velocity.*(sin(pi-gamma)).^2./thetaDerivative.*sign(thetaDerivative);
    rSAT1 = velocity.*sin(pi-gamma)./thetaDerivative.*sign(thetaDerivative);
    % r0SAT2 = velocity.*(sin(pi-gamma1)).^2./thetaDerivative1.*sign(thetaDerivative1);
    % rSAT2 = velocity.*sin(pi-gamma1)./thetaDerivative1.*sign(thetaDerivative1);


    %% only valid ranges are included 
    inds = find(r0SAT1(:) > 0); 

    r0SAT1 = r0SAT1(inds); 
    rcvLocsInterp = rcvLocsInterp(inds, :); 
    timeVec = whaleTime(inds); 
    whaleBearingFit = whaleBearingFit(inds); 
    gamma = gamma(inds); 

    %% 

    % true Range (for the source only) 
    % trueRange = sqrt((track.sourceLocation(:,1)-track.receiverInterp(:,1)).^2+(track.sourceLocation(:,2)-track.receiverInterp(:,2)).^2);

    % Estimated source locations 
    sourceLocationEstimates.xSAT = rcvLocsInterp(1:end,1) + r0SAT1./sin(gamma).*sind(whaleBearingFit(1:end));
    sourceLocationEstimates.ySAT = rcvLocsInterp(1:end,2) + r0SAT1./sin(gamma).*cosd(whaleBearingFit(1:end));
    % 
    % sourceLocationEstimates.xSAT1 = rcvLocsInterp(2:end,1) + r0SAT2./sin(gamma1).*sind(expectedMigration1(2:end));
    % sourceLocationEstimates.ySAT1 = track.receiverInterp(2:end,2) + r0SAT2./sin(gamma1).*cosd(expectedMigration1(2:end));


    % Get LLS-fitted source locations 
    coefficient = polyfit(timeVec(1:end),sourceLocationEstimates.xSAT,1);
    sourceLocationEstimates.xMotionSATLinear = polyval(coefficient,timeVec(1:end));
    coefficient = polyfit(timeVec(1:end),sourceLocationEstimates.ySAT,1);
    sourceLocationEstimates.yMotionSATLinear = polyval(coefficient,timeVec);


    % %not using these
    % coefficient = polyfit(timeVec(2:end),sourceLocationEstimates.xSAT1,1);
    % sourceLocationEstimates.xMotionSATLinear1 = polyval(coefficient,timeVec(2:end));
    % coefficient = polyfit(timeVec(2:end),sourceLocationEstimates.ySAT1,1);
    % sourceLocationEstimates.yMotionSATLinear1 = polyval(coefficient,timeVec(2:end));

    sourceLocationEstimates.xMeanSATLinear = mean(sourceLocationEstimates.xSAT);
    sourceLocationEstimates.yMeanSATLinear = mean(sourceLocationEstimates.ySAT);
    % 
    % sourceLocationEstimates.xMeanSATLinear1 = mean(sourceLocationEstimates.xSAT1);
    % sourceLocationEstimates.yMeanSATLinear1 = mean(sourceLocationEstimates.ySAT1);

    rExpected.SAT = sqrt((sourceLocationEstimates.xMotionSATLinear-rcvLocsInterp(1:end,1)).^2 ...
        +(sourceLocationEstimates.yMotionSATLinear-rcvLocsInterp(1:end,2)).^2);
    % rExpected.SAT1 = sqrt((sourceLocationEstimates.xMotionSATLinear1-track.receiverInterp(2:end,1)).^2 ...
    %     +(sourceLocationEstimates.yMotionSATLinear1-track.receiverInterp(2:end,2)).^2);




    %% Tracking with the Kalman filter 

    %% Initialization 

    sourceBearingRadian = (90-whaleBearing)*pi/180; %using non fitted whale bearings 
    stdRangeEstimates = 5e3;
    lemma = pi;
    initialTargetCourse = whaleBearing(2)*pi/180+lemma;
    stdTargetCourse = pi/sqrt(12);
    minTargetSpeed = 0;
    maxTargetSpeed = 1;
    meanTargetSpeed = (minTargetSpeed + maxTargetSpeed)/2;
    stdTargetSpeed = 0.3;
    % stdBearingNoise = std(track.bearingEstimatesTrueNorth-track.bearingTrueNorth);
    stdBearingNoise = 1; %estimate noise variance 

    %     if rSAT1(1)>25e3
    %         meanInitialRange.SAT = 15e3;
    %     else
    %         meanInitialRange.SAT = rExpected.SAT1(1);
    %     end
    %     if arrayInvariant.rangeEstimates(1)>25e3
    %         meanInitialRange.AI = 15e3;
    %     else
    %         meanInitialRange.AI = rExpected.AI(1);
    %     end

    %initiallize the range for the Kalman filter, using estimates from other
    %methods 
    initRange = ddist([sourceLocationEstimates.xMotionSATLinear(1) sourceLocationEstimates.yMotionSATLinear(1)], ...
                        rcvLocsInterp(1, :)); %here using the linear fitted SAT 

    initStateMatrix.Average = [initRange*cos(sourceBearingRadian(1)) initRange*sin(sourceBearingRadian(1)) 0 0]';
    % initCovarMatrix.Average = initCovarMatrix.SAT; %don't need 

    initialXPosition.Average = initRange*sin(whaleBearing(2)*pi/180);
    initialYPosition.Average = initRange*cos(whaleBearing(2)*pi/180);
    initialXVelocity.Average = meanTargetSpeed*sin(initialTargetCourse)-velocity(1)*sin(pi/2-atan(alpha(1)));
    initialYVelocity.Average = meanTargetSpeed*cos(initialTargetCourse)-velocity(1)*cos(pi/2-atan(alpha(1)));

    Pxx.Average = initRange^2*(stdBearingNoise*pi/180)^2*(cos(whaleBearing(2)*pi/180))^2+stdRangeEstimates^2*(sin(whaleBearing(2)*pi/180))^2;
    Pyy.Average = initRange^2*(stdBearingNoise*pi/180)^2*(sin(whaleBearing(2)*pi/180))^2+stdRangeEstimates^2*(cos(whaleBearing(2)*pi/180))^2;
    Pxy.Average = (stdRangeEstimates^2-initRange^2*(stdBearingNoise*pi/180)^2)*sin(whaleBearing(2))*cos(whaleBearing(2));
    Pyx.Average = Pxy.Average;
    Pxxv.Average = meanTargetSpeed^2*stdTargetCourse^2*(cos(initialTargetCourse))^2+stdTargetSpeed^2*(sin(initialTargetCourse))^2;
    Pyyv.Average = meanTargetSpeed^2*stdTargetCourse^2*(sin(initialTargetCourse))^2+stdTargetSpeed^2*(cos(initialTargetCourse))^2;
    Pxyv.Average = (stdTargetSpeed^2-meanTargetSpeed^2*stdTargetCourse^2)*sin(initialTargetCourse)*cos(initialTargetCourse);
    Pyxv.Average = Pxyv.Average;


    initialStateVectorCartesian.Average = [initialXPosition.Average;initialYPosition.Average;initialXVelocity.Average;initialYVelocity.Average];

    initialCovarMatrixCartesian.Average = [Pxx.Average Pxy.Average 0 0; ...
                                    Pyx.Average Pyy.Average 0 0; ...
                                    0 0 Pxxv.Average Pxyv.Average; ...
                                    0 0 Pyxv.Average Pyyv.Average];

     %% EKF in Cartesian coordinates        

    if strcmp(flag,'1')
        stateMatrix = initStateMatrix.Average;
        covarMatrix = initCovarMatrix.Average; 
    else
        stateMatrix = initialStateVectorCartesian.Average;
        covarMatrix = initialCovarMatrixCartesian.Average;
    end

    stateMatrixAll.Average = zeros(4,length(whaleBearing));
    stateMatrixAll.Average(:,1) = stateMatrix;

    for ii = 1:length(whaleBearing)-1
        A = [1 0 dt(ii) 0;0 1 0 dt(ii);0 0 1 0;0 0 0 1];
        B = [eye(2)*dt(ii);eye(2)];
        stateMatrixPredict = A*stateMatrix-B*u(ii,:)';
        slantRange = sqrt(stateMatrixPredict(1)^2+stateMatrixPredict(2)^2);
        betaPredict = atan2(stateMatrixPredict(2),stateMatrixPredict(1));
        tranMatrix = [sin(sourceBearingRadian(ii+1)) -cos(sourceBearingRadian(ii+1)) 0 0];
        covarMatrixPredict = A*covarMatrix*A';
        kalmanGain = covarMatrixPredict*tranMatrix'/(tranMatrix*covarMatrixPredict*tranMatrix'+bearingError*slantRange^2);
        stateMatrixAll.Average(:,ii+1) = stateMatrixPredict - kalmanGain*(tranMatrix*stateMatrixPredict);
        stateMatrix = stateMatrixAll.Average(:,ii+1);
        covarMatrix = (eye(4)-kalmanGain*tranMatrix)*covarMatrixPredict;
    end

    sourceLocationEstimates.kalmanAverage = stateMatrixAll.Average(1:2,1:(size(rcvLocsInterp, 1)))'+rcvLocsInterp;

    range.EKFCartesianAverage = sqrt((sourceLocationEstimates.kalmanAverage(:,1)-rcvLocsInterp(:,1)).^2+ ...
        (sourceLocationEstimates.kalmanAverage(:,2)-rcvLocsInterp(:,2)).^2);

    sourceLocationEstimates.xMeanEKFCartesianAverage = mean(sourceLocationEstimates.kalmanAverage(:,1));
    sourceLocationEstimates.yMeanEKFCartesianAverage = mean(sourceLocationEstimates.kalmanAverage(:,2));
    % 
    % rmse.meanLocationEKFCartesianAverage = sqrt((sourceLocationEstimates.xMeanEKFCartesianAverage-mean(sourceLocationEstimates.xMotionSATLinear))^2+ ...
    %     (sourceLocationEstimates.yMeanEKFCartesianAverage-mean(sourceLocationEstimates.yMotionSATLinear))^2);
    % 
    % rmse.rangeEKFCartesianAverage = sqrt(sum((range.EKFCartesianAverage-trueRange).^2)/length(trueRange));

    %% MPC-EKF 

    if strcmp(flag,'1')
        [initialStateVectorMPC.Average initialCovarMatrixMPC.Average] = getMPCInitialization_rev1(initStateMatrix.Average,initCovarMatrix.Average);
        [initialStateVectorLPC.Average initialCovarMatrixLPC.Average] = getLPCInitialization_rev1(initStateMatrix.Average,initCovarMatrix.Average);
    else
        [initialStateVectorMPC.Average initialCovarMatrixMPC.Average] = getMPCInitialization_rev1(initialStateVectorCartesian.Average,initialCovarMatrixCartesian.Average);    
        [initialStateVectorLPC.Average initialCovarMatrixLPC.Average] = getLPCInitialization_rev1(initialStateVectorCartesian.Average,initialCovarMatrixCartesian.Average);   
    end

    interimStateVectorMPC.Average = initialStateVectorMPC.Average;
    interimCovarMatrixMPC.Average = initialCovarMatrixMPC.Average;
    interimStateVectorLPC.Average = initialStateVectorLPC.Average;
    interimCovarMatrixLPC.Average = initialCovarMatrixLPC.Average;

    H = [0 0 1 0];
    relativeStateVectorMPC.Average = zeros(4,length(whaleBearing));
    relativeStateVectorMPC.Average(:,1) = interimStateVectorMPC.Average;
    relativeStateVectorLPC.Average = zeros(4,length(whaleBearing));
    relativeStateVectorLPC.Average(:,1) = interimStateVectorLPC.Average;

    for ii = 1:length(whaleBearing)-1
    %     w = [receiverTrajectory(ii+1,1)-receiverTrajectory(ii,1)-dt*observerVelocity(ii)*cos(observerHeading(ii)); ...
    %         receiverTrajectory(ii+1,2)-receiverTrajectory(ii,2)-dt*observerVelocity(ii)*sin(observerHeading(ii)); ...
    %         observerVelocityChange(ii,:)'];
        w = [dt(ii)*eye(2);eye(2)]*u(ii,:)';
        [s1MPC.Average,s2MPC.Average,s3MPC.Average,s4MPC.Average] = getMPCcomponent_rev1(interimStateVectorMPC.Average,w,dt(ii));
        [s1LPC.Average,s2LPC.Average,s3LPC.Average,s4LPC.Average] = getLPCcomponent_rev1(interimStateVectorLPC.Average,w,dt(ii));
        predictStateVectorMPC.Average = getPredictStateVectorMPC_rev1(s1MPC.Average,s2MPC.Average,s3MPC.Average,s4MPC.Average,interimStateVectorMPC.Average);
        predictStateVectorLPC.Average = getPredictStateVectorLPC_rev1(s1LPC.Average,s2LPC.Average,s3LPC.Average,s4LPC.Average,interimStateVectorLPC.Average);
        stateTransitionMatrixMPC.Average = getTransitionMatrixMPC_rev1(s1MPC.Average,s2MPC.Average,s3MPC.Average,s4MPC.Average,w,dt(ii),interimStateVectorMPC.Average);
        stateTransitionMatrixLPC.Average = getTransitionMatrixLPC_rev1(s1LPC.Average,s2LPC.Average,s3LPC.Average,s4LPC.Average,w,dt(ii),interimStateVectorLPC.Average);
        predictCovarMatrixMPC.Average = stateTransitionMatrixMPC.Average*interimCovarMatrixMPC.Average*transpose(stateTransitionMatrixMPC.Average);
        predictCovarMatrixLPC.Average = stateTransitionMatrixLPC.Average*interimCovarMatrixLPC.Average*transpose(stateTransitionMatrixLPC.Average);
        kalmanGainMPC.Average = predictCovarMatrixMPC.Average*transpose(H)/(H*predictCovarMatrixMPC.Average*transpose(H)+(stdBearingNoise*pi/180)^2);
        kalmanGainLPC.Average = predictCovarMatrixLPC.Average*transpose(H)/(H*predictCovarMatrixLPC.Average*transpose(H)+(stdBearingNoise*pi/180)^2);
        relativeStateVectorMPC.Average(:,ii+1) = predictStateVectorMPC.Average+kalmanGainMPC.Average*(whaleBearing(ii+1)*pi/180-H*predictStateVectorMPC.Average);
        relativeStateVectorLPC.Average(:,ii+1) = predictStateVectorLPC.Average+kalmanGainLPC.Average*(whaleBearing(ii+1)*pi/180-H*predictStateVectorLPC.Average);
        interimStateVectorMPC.Average = relativeStateVectorMPC.Average(:,ii+1);
        interimStateVectorLPC.Average = relativeStateVectorLPC.Average(:,ii+1);
        interimCovarMatrixMPC.Average = (eye(4)-kalmanGainMPC.Average*H)*predictCovarMatrixMPC.Average;
        interimCovarMatrixLPC.Average = (eye(4)-kalmanGainLPC.Average*H)*predictCovarMatrixLPC.Average;
    end

    relativeStateVectorCartesianMPC.Average = stateVectorMPC2Cartesian_rev1(relativeStateVectorMPC.Average);
    relativeStateVectorCartesianLPC.Average = stateVectorLPC2Cartesian_rev1(relativeStateVectorLPC.Average);

    targetTrajectoryEstimatedMPC.Average = relativeStateVectorCartesianMPC.Average(1:2,1:size(rcvLocsInterp, 1))'+rcvLocsInterp;
    targetTrajectoryEstimatedLPC.Average = relativeStateVectorCartesianLPC.Average(1:2,1:size(rcvLocsInterp, 1))'+rcvLocsInterp; 


    sourceLocationEstimates.ekfMPCAverage = targetTrajectoryEstimatedMPC.Average;
    sourceLocationEstimates.ekfLPCAverage = targetTrajectoryEstimatedLPC.Average;

    range.ekfMPCAverage = sqrt((sourceLocationEstimates.ekfMPCAverage(:,1)-rcvLocsInterp(:,1)).^2+ ...
        (sourceLocationEstimates.ekfMPCAverage(:,2)-rcvLocsInterp(:,2)).^2);
    range.ekfLPCAverage = sqrt((sourceLocationEstimates.ekfLPCAverage(:,1)-rcvLocsInterp(:,1)).^2+ ...
        (sourceLocationEstimates.ekfLPCAverage(:,2)-rcvLocsInterp(:,2)).^2);

    sourceLocationEstimates.xMeanEkfMPCAverage = mean(sourceLocationEstimates.ekfMPCAverage(:,1));
    sourceLocationEstimates.yMeanEkfMPCAverage = mean(sourceLocationEstimates.ekfMPCAverage(:,2));
    sourceLocationEstimates.xMeanEkfLPCAverage = mean(sourceLocationEstimates.ekfLPCAverage(:,1));
    sourceLocationEstimates.yMeanEkfLPCAverage = mean(sourceLocationEstimates.ekfLPCAverage(:,2));
    % 
    % rmse.meanLocationEkfMPCAverage = sqrt((sourceLocationEstimates.xMeanEkfMPCAverage-mean(track.sourceLocation(:,1)))^2+ ...
    %     (sourceLocationEstimates.yMeanEkfMPCAverage-mean(track.sourceLocation(:,2)))^2);
    % rmse.rangeEkfMPCAverage = sqrt(sum((range.ekfMPCAverage-trueRange).^2)/length(trueRange));
    % rmse.meanLocationEkfLPCAverage = sqrt((sourceLocationEstimates.xMeanEkfLPCAverage-mean(track.sourceLocation(:,1)))^2+ ...
    %     (sourceLocationEstimates.yMeanEkfLPCAverage-mean(track.sourceLocation(:,2)))^2);
    % rmse.rangeEkfLPCAverage =


    %% OUTPUT 

    wo.xEKF = sourceLocationEstimates.ekfMPCAverage(:, 1); 
    wo.yEKF = sourceLocationEstimates.ekfMPCAverage(:, 2); 
    disp('Done!'); 
end 