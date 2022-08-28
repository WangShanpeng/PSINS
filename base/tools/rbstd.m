function rstd = rbstd(data)
% Robust std calculation using median method.
%
% Prototype: rstd = rbstd(data)
% Input: data - colmun data input
% Output: rstd - std output
%
% See also  htwn, meann.
%
% Example:
%   [wn, s] = htwn(0.1, 100, 10000);  [std(wn), rbstd(wn)],

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 03/07/2022
    if size(data,1)==1, data=data';  end  % if row-vector
    rstd = data(1,:)';
    for k=1:size(data,2)
        rstd(k) = median(abs(data(:,k)-median(data(:,k))))/0.6745;
    end
