function data = cosfading(data, t0, t1, m0, isfading)
% Data within [t0,t1] multiply a factor, witch following cos(0,pi) pattern,
% fading from 1 to 0, or growing from 0 to 1.
%
% Prototype: data = cosfading(data, t0, t1, m0, isfading)
% Inputs: data - input data, whose last column should be time index
%         t0,t1 - time interval
%         m0 - data mean within [t0, t1]
%         isfading - =1 for fading from 1 to 0; =0 for growing from 0 to 1 
% Outputs: data - output data
%
% See also  datacut, datahalf, setvals.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 02/01/2024
    if nargin<5, isfading=1; end
    idx0 = find(data(:,end)>=t0, 1,'first');
    idx1 = find(data(:,end)<=t1, 1,'last');
    if nargin<4, m0=mean(data(idx0:idx1,:)); end
    if length(m0)<size(data,2)-1, m0=mean(data(idx0:idx1,:)); end
    csfd = (cos(pi/(idx1-idx0)*(0:(idx1-idx0)))'+1)/2;
    if isfading==0, csfd = flipud(csfd);  end
    for k=1:size(data,2)-1
        data(idx0:idx1,k) = (data(idx0:idx1,k)-m0(k)).*csfd+m0(k);
    end
    