function gps = gpsload(fname, description)
% There may exist different type of GPS log files, code and call
% this function to read specific GPS log file.
%
% See also  gpsplot, imuplot.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/03/2014
    switch(description)
        case 'leador',
            gps = gpsloadleador(fname);
        case 'WangJun',
            gps = gpsloadWangJun(fname);
    end
        
function gps = gpsloadleador(fname)
global glv
	dd = load(fname);
	gps=dd; len=length(gps);
	for k=2:len
    	kk = gps(k,2)-gps(k-1,2);
    	if kk>1
        	gps = [gps(1:k-1,:); gps(k-1,:); gps(k:end,:)];
        	gps(k,2) = gps(k-1,2)+1;  gps(k,6) = 0; % 将丢点精度因子置0
        	len = len+1;
        end
	end
	gps(:,3:4) = gps(:,3:4)*glv.deg;
	figure, plot([0;diff(dd(:,2))-1]), hold on, plot(gps(:,6),'r'), grid on, hold off

function gps = gpsloadWangJun(fname)
global glv
    gps = load(fname);
% 	gps = [ gps(:,1:2), gps(:,3)*glv.deg+gps(:,4)*glv.min+gps(:,5)*glv.sec, ...
%         gps(:,6)*glv.deg+gps(:,7)*glv.min+gps(:,8)*glv.sec, gps(:,9:end) ];
    gps = [ gps(:,1:2), dms2r(gps(:,3:5)), dms2r(gps(:,6:8)), gps(:,9:end) ];