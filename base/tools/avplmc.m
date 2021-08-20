function avplmc(avp1, avp2, idx)
% One column in AVP to Move & Compare:
%
% Prototype: avplmc(avp1, avp2, idx)
% Inputs: avp1, avp2 - AVP
%         idx - AVP column index 1~15
%
% See also lmc.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 15/05/2021
global glv
    switch idx
        case {1,2,3, 7:8}
            unit = glv.deg;
        case {4,5,6, 9}
            unit = 1;
        case {10,11,12}
            unit = glv.dph;
        case {13,14,15}
            unit = glv.ug;
    end
    lmc(avp1(:,[idx,end]), avp2(:,[idx,end]), unit, 0);
    