
function w = getMMSEloc(w)
        disp('Getting MMSE stationary estimate..'); 
        %initialization 
        rcv = w.rcvLocs; 
        beta1 = w.trueBearings; 
        x1 = rcv(1, 1) - 100e3; 
        x2 = x1 + 200e3; 
        y1 = rcv(1, 2) - 100e3; 
        y2 = y1 + 200e3; 

        %recursively estimate the location by "casting nets" and narrowing
        %down the search box 
        
        [x_hat, y_hat] = estMMSE(rcv, beta1, 2000, x1, x2, y1, y2); %coarse 
        [x_hat, y_hat] = estMMSE(rcv, beta1, 500, ...
                                x_hat - 40e3, x_hat + 40e3,...
                                y_hat - 40e3, y_hat + 40e3); %fine 
        [x_hat, y_hat] = estMMSE(rcv, beta1, 100, ...
                                x_hat - 20e3, x_hat + 20e3,...
                                y_hat - 20e3, y_hat + 20e3); %fine 

        w.xMMSE = x_hat; 
        w.yMMSE = y_hat; 

        disp('Done!'); 
end
