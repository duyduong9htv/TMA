function w = getErrorGrid(w)
        disp('Getting MMSE stationary estimate...and error grid...'); 
        %initialization 
        rcv = w.rcvLocs; 
        beta1 = w.trueBearings; 
        x1 = rcv(1, 1) - w.searchRadius; 
        x2 = x1 + 2*w.searchRadius; 
        y1 = rcv(1, 2) - w.searchRadius; 
        y2 = y1 + 2*w.searchRadius; 


        %actual MMSE localization function 
        function [x_hat, y_hat, error_grid] = estMMSE(rcv, bearings, res, x1, x2, y1, y2)
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

        [x_hat, y_hat, error_grid] = estMMSE(rcv, beta1, 10, x1, x2, y1, y2); %coarse 
        

        w.xMMSE = x_hat; 
        w.yMMSE = y_hat; 
        w.error_grid = error_grid; 
        disp('Done!'); 
end
