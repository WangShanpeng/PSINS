function rvpplot(rvp, p, clm)
% Standard deviation of Kalman vel&pos measurement noise plot.
%
% Prototype: rvpplot(rvp, p, clm)
% Inputs: rvp - =[Zk(vel), Zk(pos), Var(vel], Var(pos), t]
%         p - if rvp=[Zk(vel), Zk(pos), t], then p=[Var(vel], Var(pos), t] 
%
% See also  kfplot, inserrplot, kffile.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 02/03/2020
global glv
    if nargin==3, rvp=[rvp(:,clm),p(:,[clm,end])]; end  % rvpplot(vp, var_vp, clm)
    if nargin==2, rvp=[rvp(:,1:end-1), p]; end   % rvpplot(vp, var_vp)
    myfigure
    if size(rvp,2)>7
        subplot(221), plot(rvp(:,end), rvp(:,1:3)); xygo('Zvel / m/s')
        subplot(223), plot(rvp(:,end), sqrt(rvp(:,7:9))); xygo('std(vel) / m/s')
        subplot(222), plot(rvp(:,end), [rvp(:,4:5)*glv.Re,rvp(:,6)]); xygo('Zpos / m')
        subplot(224), plot(rvp(:,end), sqrt([rvp(:,10:11)*glv.Re^2,rvp(:,12)])); xygo('std(pos) / m')
    else   % rvpplot(p, var_p, clm)
        subplot(211), plot(rvp(:,end), [rvp(:,1:2)*glv.Re,rvp(:,3)]); xygo('Zpos / m')
        subplot(212), plot(rvp(:,end), sqrt([rvp(:,4:5)*glv.Re^2,rvp(:,6)])); xygo('std(pos) / m')
    end