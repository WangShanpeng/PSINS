function errstplot(err, state, maxst)
% AVP error & KF Runing-state plot.
%
% Prototype: errstplot(err, state, maxst)
% Inputs: err - AVP error
%         state - runing state
%         maxst - max state
%          
% See also  avpcmpplot, stateplot.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 26/07/2021
    global glv
    nn = (size(err,2)-1)/3;
    myfig;
    if nn==3
    phi_mu = 'phi';
	subplot(nn+1,1,1), plot(err(:,end), err(:,1:3)/glv.min); xygo(phi_mu); mylegend([phi_mu,'x'],[phi_mu,'y'],[phi_mu,'z']);
    end
    if nn>=2
    subplot(nn+1,1,nn-1), plot(err(:,end), err(:,end-6:end-4)); xygo('dV'); mylegend('dVE','dVN','dVU');
    end
    subplot(nn+1,1,nn), plot(err(:,end), [[err(:,end-3:end-2)]*glv.Re,err(:,end-1)]); xygo('dP'); mylegend('dlat','dlon','dH');

    [n, m] = size(state);
    if m==1,  t = (1:n)';
    else      t = state(:,end); state = state(:,end-1); end
    sstate = [];
    for k=0:31
        tmp = bitand(state,2^k);
        mtmp = max(tmp);
        if mtmp>0, sstate = [sstate, tmp/mtmp*(k+1)]; end
    end
    subplot(nn+1,1,nn+1), plot(t, sstate, '*'); xygo('Running-state');
    maxst0 = max(max(sstate));
    if nargin<3, maxst=0; end;
    maxst = max(maxst0,maxst)+0.2;
    ylim([0,maxst]);
