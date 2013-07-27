
function w = getMMSEloc(w)
        disp('Getting MMSE stationary estimate..'); 
        %initialization 
        rcv = w.rcvLocs; 
        beta1 = w.trueBearings; 
        x1 = rcv(1, 1) - 100e3; 
        x2 = x1 + 200e3; 
        y1 = rcv(1, 2) - 100e3; 
        y2 = y1 + 200e3; 


        %actual MMSE localization function 
        function [x_hat, y_hat] = estMMSE(rcv, bearings, res, x1, x2, y1, y2)
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
