function rcvLocsInterp = getRcvLocsInterp(whaleTime, rcvTime, rcvLocs)
%function rcvLocsInterp = getRcvLocsInterp(whaleTime, rcvTime, rcvLocs)
% gets the interpolated locations of the receivers at the time of the whale call. 
%This is to avoid using the same receiver location for all whale calls recorded within a ping 
% INPUT			: 	-whaleTime: actual whale time, in seconds from 00:00:00
%					-rcvTime : time of recording of receiver location, also called ping time, eg 051300
%					-rcvLocs : GPS location of the receiver, at ping time 
% OUTPUT		: 	-rcvLocsInterp : interpolated locations of receiver array. 

rcvLocsInterp = zeros(size(rcvLocs)); 
rcvLocsInterp(1, :) = rcvLocs(1, :); 
rcvLocsInterp(end,:) = rcvLocs(end,:); 

for ii = 2:(length(whaleTime) - 1)
	inds1 = find(rcvTime(:)<=whaleTime(ii)); 
	inds2 = find(rcvTime(:)>=whaleTime(ii)); 
	rcvTimeBefore = rcvTime(inds1(end)); 
    if ~isempty(inds2)
        rcvTimeAfter = rcvTime(inds2(1)); 
       rcvLocsAfter = rcvLocs(inds2(1), :); 
    end 
	rcvLocsBefore = rcvLocs(inds1(end), :); 
    if rcvTimeAfter-rcvTimeBefore == 0
        rcvLocsInterp(ii, :) = rcvLocs(ii, :); 
    elseif isempty(inds2)
        rcvLocsInterp(ii, :) = rcvLocs(ii, :); 
    else
        rcvLocsInterp(ii, :) = (whaleTime(ii) - rcvTimeBefore)/(rcvTimeAfter - rcvTimeBefore)*(rcvLocsAfter - rcvLocsBefore) + rcvLocsBefore; 
    end
end


end 

