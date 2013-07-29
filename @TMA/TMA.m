classdef TMA 
%Target Motion Analysis Summary of this class goes here
%  
%Class for performing the 3 methods of localization using bearings-only 
%Synthetic Aperture, MMSE (assuming stationary), and Kalman filter in
%Modified Polar Coordinates

   
    properties
        arrayHeading;
        relativeBearings;
        whaleTime; 
        rcvLocs; 
        rcvTime
        trueBearings;
        dateTime;
        piecewisewindow; %in minutes 
        timeBounds;
        freqBounds;
        duration;
        lastUpdate;
        pingID;
        trackName;
        groupID;
        pingHeading;
        f0;
        f1;
        tStart_2;
        duration_2;
        xSATinst; 
        ySATinst; 
        xSATtrack; 
        ySATtrack; 
        
        xMMSE; 
        yMMSE; 
        
        xEKF; 
        yEKF;
        
        trackTimeWindowed; 
        
        error_grid;
        searchRadius; 
        
        %control 
        piecewisefit; 
        L; %minimum synthetic aperture for tracking 
        Lt; %time windowed for MSE tracking, in minutes 
        
    end
    
    methods
        %class constructor 
        function w = TMA(rcv, rcvTime, measuredBearings,measurementTime, arrayHeading)
            w.rcvLocs = rcv; 
            w.rcvTime = rcvTime; 
            w.trueBearings = measuredBearings; 
            w.whaleTime = measurementTime; 
            w.arrayHeading = arrayHeading; 
            w.piecewisewindow = 30; %by default assuming a 30 minute piecewiselinear window 
            w.piecewisefit = 1; %by default, use piecewise linear fit, assuming linear dependence in a window of about 20 minutes
            %if w.piecewisefit = 0, the linear fit is used. 
            w.L= 400;  %by default, assuming an aperture threshold = 400 m (near field ~ 50 km)
            w = w.updateRelativeBearings; 
            w.Lt = 5; 
        end
        
        
               
        % reset all results          
        function w = reset(w)
            w.xSATinst = []; w.ySATinst = []; 
            w.xSATtrack = [];w.ySATtrack = []; 
            w.xMMSE = []; w.yMMSE = []; 
            w.xEKF = []; w.yEKF = []; 
            
            disp('Results all reset!'); 
        end
        

        
        
        
        
    end
end

