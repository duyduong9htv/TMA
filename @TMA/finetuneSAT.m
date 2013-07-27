
function w = finetuneSAT(w) %run after running getSATresults
     N = length(w.xSATinst); 
     time1 = w.whaleTime(1:N);
     [~, ~, indsx] = selectPolygon(time1, w.xSATinst, 'Time', 'UTM x estimates'); 


     [~, ~, indsy] = selectPolygon(time1, w.ySATinst, 'Time', 'UTM y estimates'); 

     inds = intersect(indsx, indsy); 

     x1 = w.xSATinst(inds); 
     y1 = w.ySATinst(inds); 
     time1 = w.rcvTime(1:N); 
     t1 = time1(inds); 
     Px = polyfit(t1, x1, 1); 
     w.xSATtrack = polyval(Px, w.rcvTime(1:N)); 
     Py = polyfit(t1, y1, 1); 
     w.ySATtrack = polyval(Py, w.rcvTime(1:N)); 

end
            