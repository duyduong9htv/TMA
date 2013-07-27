 function w = updateRelativeBearings(w)
    for k = 1:length(w.trueBearings)
%                 trueBearing = calBearings(po(k).srcUTM, po(k).rcvUTM); 
        theta = w.trueBearings(k);
        w.relativeBearings(k) = w.arrayHeading(k) + sign(180 - w.arrayHeading(k))*90 - theta; 
        w.relativeBearings(k) = asind(sind(w.relativeBearings(k))); %restrict to between -90 and 90 degrees
    end

end