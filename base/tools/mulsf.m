function x = mulsf(x, sf, clm)
% Multiply scale factors.
%
% Prototype:  x = mulsf(x, sf, clm)
% Inputs: x_in - input data with bias
%         sf - scale factor to multiply
%         clm - data column to delete bias
% Outputs: x - output data with no bias
%
% See also  delbias, imudeldrift.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 10/08/2022 
    if nargin<3, clm=1:length(sf); end
    if length(sf)==1, sf=repmat(sf,1,length(clm)); end
    for k=1:length(clm)
        x(:,clm(k)) = x(:,clm(k)) * sf(k);
    end
