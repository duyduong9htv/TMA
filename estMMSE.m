function [x_hat, y_hat] = estMMSE(rcv, bearings, res, x1, x2, y1, y2)
% function [x_hat, y_hat] = estMMSE(rcv, bearings, res, x1, x2, y1, y2)
% finds a stationary location x_hat, y_hat based on bearing-only tracking
% approach and input true bearings (w.r.t) true Norths from receiver
% locations rcv ( N x 2 )
    error_grid = zeros(length(x1:res:x2), length(y1:res:y2)); 

    x_ind = 0; 
    MSE = 1e100; 
    for xx = x1:res:x2
        x_ind = x_ind + 1; 
        y_ind = 0; 
        for yy = y1:res:y2
            y_ind = y_ind + 1; 
            angle1 = atand((yy - rcv(:, 2))./(xx - rcv(:, 1))); 
            angle1 = angle1 - 90*(1 - sign(xx - rcv(:, 1)));
            beta = 90 - angle1; 
            error = sum((beta - bearings).^2); 
            if error<MSE
                MSE = error; 
                x_hat = xx; y_hat = yy; 
            end    
            error_grid(x_ind, y_ind) = error; 
        end
    end
end 
