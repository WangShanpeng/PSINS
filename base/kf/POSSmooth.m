function vpOut = POSSmooth(vp, gnss, nSec, isfig)
% POS data smooth via INS-VelPos & GNSS-VelPos.
%
% Prototype: vpOut = POSSmooth(vp, gnss, nSec, isfig)
% Inputs: vp - INS Vel&Pos input array = [vn,pos,t]
%         gnss - GNSS Vel&Pos input array = [vn,pos,t]
%         nSen - smooth span in second
%         isfig - figure flag
% Output: vpOut - INS Vel&Pos output after smooth
%
% See also  POSFusion, POSProcessing, smoothol, fusion.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 18/10/2021
    if nargin<4, isfig=1; end
    if nargin<3, nSec=10; end
	err = gnss(:,end-1) - interp1(vp(:,end), vp(:,end-1), gnss(:,end));
	gnss = gnss(~isnan(err),:);  err = gnss;  errs = err;
    nSec = fix(nSec/(gnss(end,end)-gnss(1,end))*length(gnss));  % smooth points
    for k=1:size(gnss,2)-1
        err(:,end-k) = gnss(:,end-k) - interp1(vp(:,end), vp(:,end-k), gnss(:,end));
        [~, errs(:,end-k)] = smoothol(err(:,end-k), nSec, 2, 0);
    end
    idx = ~isnan(interp1(errs(:,end), errs(:,end-k), vp(:,end)));
    vpOut = vp;
    for k=1:size(gnss,2)-1
        vpOut(idx,end-k) = vp(idx,end-k) + interp1(errs(:,end), errs(:,end-k), vp(idx,end), 'spline');
    end
    if isfig==1
        eth = earth(gnss(1,end-3:end-1)');
        myfig
        if size(gnss,2)==7, subplot(211); end
        plot(err(:,end), [err(:,end-3)*eth.RMh, err(:,end-2)*eth.clRNh, err(:,end-1)]); xygo('dP');
        plot(errs(:,end), [errs(:,end-3)*eth.RMh, errs(:,end-2)*eth.clRNh, errs(:,end-1)],'-.','linewidth',2);
        if size(gnss,2)==7
            subplot(212); 
            plot(err(:,end), err(:,end-6:end-4)); xygo('V'); plot(errs(:,end), errs(:,end-6:end-4), '-.','linewidth',2)
            avpcmpplot(gnss, vpOut(:,end-6:end), 'vp');
        else
            avpcmpplot(gnss, vpOut(:,end-3:end), 'p');
        end
    end
