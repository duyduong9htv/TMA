function out = piecewiselinearfit(whaleTime, whaleBearing, windowLength)
% function out = piecewiselinearfit(whaleTime, whaleBearing)
% performs a piece wise linear fitting on a series of bearings measured at
% different time instances 
%input      : 
%           -whaleTime(Nx1)
%           -whaleBearing (Nx1)
%           -windowLength (length of window time for the 'piece' linear fit
%           (in minutes)
%OUTPUT     : 
%           -out (Nx1) 



fitMatrix  = zeros(length(whaleBearing)); 

for k = 1:length(whaleTime)
    whaleTime1 = whaleTime((k):end); 
    whaleBearing1 = whaleBearing(k:end); 
    inds = find((whaleTime1(:) - whaleTime(k))<windowLength*60)
    t = whaleTime1(inds); 
    b = whaleBearing1(inds); 
    if ~isempty(inds)
        P1 = polyfit(t, b, 1); 
        b1 = polyval(P1, t); 
        fitMatrix(k, k:(k+length(inds) - 1)) = b1; 
    else 
        a = 0; 
    end     
end

out = zeros(1, length(whaleTime)); 

for k = 1:size(fitMatrix, 2)
    inds = fitMatrix(:, k)>0; 
    out(k) = mean(fitMatrix(inds, k)); 
end




end
