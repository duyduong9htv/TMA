addpath /Users/dtran/luars
LUARSMatlabPath; 
LUARSDatabaseConnector; 


addpath /Users/dtran/Research/whale_localization_codes/bearings_only
addpath /Users/dtran/Research/whale_localization_codes/bearings_only/EKFcodes/

%% querying line  
whales = WhaleQuery('frequency', [50 1000], 'dateTime',...
    {'2006-10-02 10:00:00', '2006-10-03 10:00:00'});
% note that time is in GMT, not EDT 

% whales = WhaleQuery('track', '571_7'); 


%% getting rid of empty bearing measurements
for do = 1
    whales1 = [];
    for ii = 1:size(whales,2)
        if ~isempty(whales(ii).trueBearing)
            whales1 = [whales1 whales(ii)];
        end
    end

    whales = whales1;clear whales1; 


    whaleTime = []; 
    whaleBearing = []; 
    for ii = 1:size(whales, 2)
        secs = dateTimeToSeconds(whales(ii).dateTime);
        whaleTime = [whaleTime; secs]; 
        if whales(ii).trueBearing <0
            whales(ii).trueBearing = whales(ii).trueBearing + 360; 
        end
        whaleBearing = [whaleBearing; whales(ii).trueBearing]; 
    end
end 

%% plot bearings vs. time 

figure(10); clf; hold on; box on; 
setFigureAuto; setFont(18); 
plot( whaleTime, whaleBearing, 'k*'); 
set(gca, 'xtick', [0:1800:30*3600]); 
xtl = get(gca, 'xtick'); 
set(gca, 'xticklabel', xtl/3600); 
xlabel('Hours (EDT)'); 
ylabel('Bearings'); 


%% draw a polygon to select the scattered bearings to track 
[xPolygon,yPolygon]=getline(gcf,'closed')
in = find(inpolygon(whaleTime, whaleBearing, xPolygon, yPolygon))
hold on; 
plot(whaleTime(in), whaleBearing(in), 'ro'); 
whaleTime = whaleTime(in); 
whaleBearing = whaleBearing(in); 

%% plot selected bearing sequence 
figure(2); clf; hold on; box on; 
setFigureAuto; setFont(18); 
plot( whaleTime, whaleBearing, 'k*'); 
set(gca, 'xtick', [0:1800:26*3600]); 
xtl = get(gca, 'xtick'); 
set(gca, 'xticklabel', mod(xtl/3600, 24)); 
xlabel('Hours (GMT)'); 
ylabel('Bearings'); 

%% localizing
%% get rcv locations 
rcvLocs = []; 
rcvTime = [];
rcvHeadings = []; 
for ii = 1:length(in)
    kk = in(ii); 
    id = whales(kk).pingID;
    ping = PingQuery('id', id); 
    [x, y, utmzone] = deg2utm(ping.receiverLatLng(1), ping.receiverLatLng(2)) ; 
    rcvTime = [rcvTime; dateTimeToSeconds(ping.startTime)]; 
    rcvLocs = [rcvLocs; x y];        
    rcvHeadings = [rcvHeadings; ping.headingAvg];
end

%% initialize TMA object
rcvLocsInterp = getRcvLocsInterp(whaleTime, rcvTime, rcvLocs);

w = TMA(rcvLocsInterp, rcvTime, whaleBearing, whaleTime, rcvHeadings); %initialize Target Motion Analysis object 
w.arrayHeading = rcvHeadings; 
w.piecewisefit = 1 ; %disable piecewise linear fit. If want to do that set w.piecewisefit = 1; 

%% get results 
w  = w.getMMSEloc;
% w = w.getSATresults; 

w = w.getSATresultsThreshold; 
w = w.finetuneSAT;
% w = w.getEKFresults ;

w.plotResults; 


%% plotting 

figure; plot2dd(w.rcvLocs, '--k'); 
hold on; 
plot(w.xSATinst, w.ySATinst, '*'); 
plot(w.xEKF, w.yEKF, 'k*'); 
plot(w.xMMSE, w.yMMSE, 'r*', 'linewidth', 2); 
% plot(w.xEKF(1), w.yEKF(1), 'g*', 'linewidth', 2); 
axis equal 
plot(w.xSATtrack, w.ySATtrack, '--r', 'linewidth', 2); 

set(gca, 'xtick', (w.rcvLocs(1,1)-100e3):10e3:(w.rcvLocs(1,1)+100e3)); 
set(gca, 'ytick', (w.rcvLocs(1,2)-100e3):10e3:(w.rcvLocs(1,2)+100e3)); 
xtl = get(gca, 'xtick'); 
ytl = get(gca, 'ytick'); 
set(gca, 'xticklabel', (xtl-w.rcvLocs(1,1))/1000); 
set(gca, 'yticklabel', (ytl-w.rcvLocs(1, 2))/1000); 

%referencing source location
set(gca, 'xtick', (ping.srcUTM(1)-100e3):10e3:(ping.srcUTM(1)+100e3)); 
set(gca, 'ytick', (ping.srcUTM(2)-100e3):10e3:(ping.srcUTM(2)+100e3)); 
xtl = get(gca, 'xtick'); 
ytl = get(gca, 'ytick'); 
set(gca, 'xticklabel', round((xtl-ping.srcUTM(1))/1000)); 
set(gca, 'yticklabel', round((ytl-ping.srcUTM(2))/1000)); 

setFont(18); setFigureAuto; 
contourlines 
plot_bathycontour_dd(overlayfiles, 'k'); 
xlabel('Eastings from rcv start (km)'); 
ylabel('Northings from rcv start (km)'); 


