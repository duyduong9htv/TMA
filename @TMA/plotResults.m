function w = plotResults(w)
    figure; plot2dd(w.rcvLocs, '--k'); 
    hold on; 
    plot(w.xSATinst,w.ySATinst, '*'); 
    axis equal; 
    setFont(18); setFigureAuto; 
    x0 = w.rcvLocs(1, 1); 
    set(gca, 'xtick', (x0-100e3):10e3:(x0+100e3)); 
    xtl = get(gca, 'xtick'); 
    set(gca, 'xticklabel', round((xtl - x0)/1000)); 

    y0 = w.rcvLocs(1, 2); 
    set(gca, 'ytick', (y0-100e3):10e3:(y0+100e3)); 
    ytl = get(gca, 'ytick'); 
    set(gca, 'yticklabel', round((ytl - y0)/1000)); 
    xlabel('Eastings (km)'); ylabel('Northings (km)'); 

    plot(w.xSATtrack, w.ySATtrack, '--r', 'linewidth', 2); 
    plot(w.xSATtrack(1), w.ySATtrack(1), 'go', 'markersize', 5, 'linewidth',2 )
%             plot(w.xEKF, w.yEKF, 'k*'); 

end