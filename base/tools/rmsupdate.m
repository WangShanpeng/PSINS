function [rmsdata, meandata, stddata] = rmsupdate(rmsdata, meandata, err, k)
% Calculating rms, mean & std using interation method.
%
% Prototype: [rmsdata, meandata, stddata] = rmsupdate(rmsdata, meandata, err, k)
% Inputs: rmsdata - rms data in
%         meandata - std data in
%         err  - data to be calculated, the last column is time tag
%         k - interation count
% Outputs: rmsdata - rms data out
%          meandata - std data out
%          stddata - std data out
%
% Example£º
%   r=[]; m=[]; for k=1:100, [r, m, s] = rmsupdate(r, m, appendt(-1+randn(10,1)), k); end
%   myfig, plot(r(:,end), [r(:,1:end-1), m(:,1:end-1), s(:,1:end-1)]); xygo;
%
% See also  avperrstd, cep, rms2cep.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 27/06/2022
    if nargin==3, k=err; err=meandata; end   % rmsdata = stdupdate(rmsdata, data, k)
    if k==1
        rmsdata = err;  meandata = err;
        rmsdata(:,1:end-1) = abs(err(:,1:end-1));
    else
        rmsdata(:,1:end-1) = sqrt((rmsdata(:,1:end-1).^2*(k-1) + err(:,1:end-1).^2)/k);
        if nargout>1
            meandata(:,1:end-1) = (meandata(:,1:end-1)*(k-1)+err(:,1:end-1))/k;
        end
    end
    if nargout==3
        stddata = sqrt(rmsdata.^2 - meandata.^2);
    end
