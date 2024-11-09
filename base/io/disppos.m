function disppos(pos, t)
% Display latitude/longitude in degree, ang height in meter.
%
% Prototype: disppos(pos, t)
% Inputs: pos - position array.
%         t - the nearest time tag
%          
% See also  gpsplot, pos2bd.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 04/08/2024
global glv
    if size(pos,2)>3
        if nargin<2, t=-inf; end
        idx = find(pos(:,end)<t,1,'first');
        if isempty(idx), idx=1; end
        t = pos(idx,end);  pos = pos(idx,end-3:end-1);
        fprintf('\tLLH,T = [ %.8f, %.8f, %.4f ], %.4f\n', pos(1)/glv.deg, pos(2)/glv.deg, pos(3), t);
    else
        fprintf('\tLLH = [ %.8f, %.8f, %.4f ]\n', pos(1)/glv.deg, pos(2)/glv.deg, pos(3));
    end
