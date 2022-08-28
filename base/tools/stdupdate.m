function [stddata, meandata] = stdupdate(stddata, meandata, data, k)
% Calculating std & mean using interation method.
%
% Prototype: [stddata, meandata] = stdupdate(stddata, meandata, data, k)
% Inputs: stddata - std data in
%         meandata - std data in
%         data  - data to be calculated, the last column is time tag
%         k - interation count
% Outputs: stddata - std data out
%          meandata - std data out
%
% Example£º
%   s=[]; m=[]; for k=1:100, [s, m] = stdupdate(s, m, appendt(randn(10,2)), k); end
%   myfig, plot(s(:,end), [s(:,1:end-1), m(:,1:end-1)]); xygo;
%
% See also  avperrstd, cep, rms2cep.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 27/06/2022
    if nargin==3, k=data; data=meandata; end % stddata = stdupdate(stddata, data, k)
    if k==1
        stddata = data;  meandata = data;
        if nargout==2
            stddata(:,1:end-1) = sqrt(data(:,1:end-1).^2);
        end
    else
        stddata(:,1:end-1) = sqrt((stddata(:,1:end-1).^2*(k-1) + data(:,1:end-1).^2)/k);
        if nargout==2
            meandata(:,1:end-1) = (meandata(:,1:end-1)*(k-1)+data(:,1:end-1))/k;
        end
    end

