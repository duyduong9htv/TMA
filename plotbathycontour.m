%---------------------------------------------------------------------------
%
% TITLE:    CREATE_UT_OVERLAY
% AUTHOR:   Lilimar A. Ruhlmann
% DATE:	    January 17, 2000
%
% DESCRIPTION: Creates overlay file from University of Texas file format.
%		
%---------------------------------------------------------------------------
%
function w = plotbathycontour(w, color)
%
% Define global variables.
%
%global  grid_inc
%global  grid_xmin
%global  grid_xmax
%global  grid_ymin
%global  grid_ymax

%
% Declare some constant variables.
%
%xrange = grid_xmax-grid_xmin;
%yrange = grid_ymax-grid_ymin;
%
% Start reading input files.

%

contourlines; 

for ii=1:size(filename,1)
	fid = fopen(deblank(filename(ii,:)),'rt');  
	%fid
	start_flag = 0;
	string1 = fscanf(fid,'%s',1);
	string2 = fscanf(fid,'%s',1);

	f1=1;
	while (feof(fid) == 0)    
   		if (strcmp(string1,'>'))
       			if (start_flag)
				plot(utm_xvalue(1,1:nvalues), utm_yvalue(1,1:nvalues), 'Color', color, 'Linestyle', '-','linewidth',1);
				%line(datax(1,1:nvalues), datay(1,1:nvalues), 'Color', 'kw', 'Linestyle', '-','linewidth',0.5);
       			end
       			nvalues = 0;
       			utm_xvalue =0;
       			utm_yvalue = 0;
       			start_flag = 1;
   		else
       			nvalues = nvalues+1;
       			utm_xvalue(nvalues) = str2num(string1);
       			utm_yvalue(nvalues) = str2num(string2);
%       		ratio = (utm_xvalue-grid_xmin)/xrange;
%       		datax(1,nvalues) = ratio*xsize+1;
%       		ratio = (utm_yvalue-grid_ymin)/yrange;
%       		datay(1,nvalues) = ysize-(ratio*ysize+1);
   		end
   		string1 = fscanf(fid,'%s',1);
   		string2 = fscanf(fid,'%s',1);
	end
	plot(utm_xvalue(1,1:nvalues), utm_yvalue(1,1:nvalues), 'Color', color, 'Linestyle', '--','linewidth',1);
	fclose(fid);
end
