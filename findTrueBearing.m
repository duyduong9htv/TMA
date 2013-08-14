function [bearing_port, bearing_starboard] = findTrueBearing(relBearing, rcvHeading)
% function [bearing_port, bearing_starboard] = findTrueBearing(relBearing, rcvHeading)
% finds the true bearing w.r.t true North of a signal after its relative
% bearing has been obtained by beamforming. 
% INPUTS: 
%           relBearing  : relative bearing, in degrees. 
%           rcvHeading  : current receiver array heading 
% OUTPUTS: 
%           trueBearing : true bearing, w.r.t true North 

% trueBearing = rcvHeading + sign(180 - rcvHeading)*90 - relBearing; 

bearing_port = mod(rcvHeading  - 90 + relBearing, 360); 
bearing_starboard = mod(rcvHeading + 90 - relBearing, 360); 

end
