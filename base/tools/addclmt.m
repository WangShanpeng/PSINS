function data = addclmt(data, ts, t0, delt)
% Add a last column (time tag) to the data.
%
% Prototype: data = addclmt(data, ts, t0)
% Inputs: data - data to be added
%         ts - sampling interval
%         t0 - begin time
%         delt - delete old time tag
% Output: data - data after time added
%
% See also  adddt, appendt.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 19/10/2022
    if nargin<4, delt=0; end
    if nargin<3, t0=0; end
    if nargin<2, ts=1; end
    if delt==1, data(:,end) = []; end
    data = [data, t0+(0:size(data,1)-1)'*ts];
