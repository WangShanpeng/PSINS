function [t, tp2n, tn2p] = zeroyt(yt, isfig)
% Find the curve's zero points.
%
% Prototype: [t, tp2n, tn2p] = zeroyt(yt, isfig)
% Inputs: yt - two-column array [y, t]
%         isfig - figure flag
% Outputs: t - zero-point time tag
%          tp2n, tn2p - time tag from positive to negative or reverse.
%
% Example
%   [t, tp2n, tn2p] = zeroyt(appendt(randn(10,1),0.1),1);
%
% See also  sortt, tsetflag, tshift, tsyn, igsplot.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 15/08/2023
    y = yt(:,1); t = yt(:,2);
    idxp = yt(:,1)>0; idxn = ~idxp;
    idxp2n = idxp(1:end-1) & idxn(2:end);
    idxp=[idxp2n;0&0]; idxn=[0&0;idxp2n];
    tp2n = t(idxp)-(t(idxp)-t(idxn))./(y(idxp)-y(idxn)).*y(idxp);
    %%
    idxp = yt(:,1)>0; idxn = ~idxp;
    idxn2p = idxp(2:end) & idxn(1:end-1);
    idxn=[idxn2p;0&0]; idxp=[0&0;idxn2p];
    tn2p = t(idxn)-(t(idxp)-t(idxn))./(y(idxp)-y(idxn)).*y(idxn);
    %%
    if nargin<2, isfig=0; end
    if isfig==1
        myfig, plot(t, y, '-', tp2n, tp2n*0, '*', tn2p, tn2p*0, 'o'); xygo('val');
    end
    %%
    t = sort([tp2n; tn2p]);

