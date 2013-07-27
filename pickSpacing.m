
function [rcv1, inds] = pickSpacing(rcvLocs, L)
% pick the points for the KF/SAT such that receiver spacing is adequate for target
% to be in the near field. 
% INPUT: 
%   -rcvLocs    : receiver locations at measurement points 
%   -L          : threshold (meter) 
% OUTPUT: 
%   -rcv1       : receiver locations picked. 
%   -inds       : indices of points picked. 

    k1 = 1; 
    k = 1; 
    N = max(size(rcvLocs)); 
    rcv1 = rcvLocs(1, :); 
    inds = 1; 
    while k < N
        disp(k); 
        rcv_current = rcv1(end, :); 
        while ddist(rcv_current, rcvLocs(k1, :)) < L
            if k1 < N
                k1 = k1+1;
            else 
                break
            end            
        end
        inds = [inds; k1]; 
        rcv1 = [rcv1; rcvLocs(k1, :)]; 
        k = k1; 
        k1 = k;
    end
    
    
end

