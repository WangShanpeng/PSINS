function [pf, Pxk] = pfupdate(pf, Zk, TimeMeasBoth)
% Particle filter updating for additive normal distribution process&measure noise.
%
% Prototype: [pf, Pxk] = pfupdate(pf, Zk, TimeMeasBoth)
% Inputs: pf - particle filter structure array
%         Zk - measurement vector
%         TimeMeasBoth - described as follows,
%            TimeMeasBoth='T' (or nargin==1) for time updating only, 
%            TimeMeasBoth='M' for measurement updating only, 
%            TimeMeasBoth='B' (or nargin==2) for both time and 
%                             measurement updating.
% Outputs: pf - particle filter structure array after time/meas updating
%          Pxk - for full covariance (not diagnal)
%
% see also  pfinit, kfupdate, ckf, ukf.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/04/2022
    if nargin==1;
        TimeMeasBoth = 'T';
    elseif nargin==2
        TimeMeasBoth = 'B';
    end

    if TimeMeasBoth=='T' || TimeMeasBoth=='B'
        sQ = chol(pf.Qk+eps*eye(pf.n));
    end
	if TimeMeasBoth=='M' || TimeMeasBoth=='B'
        vRv = zeros(pf.Npts, 1);  invR = pf.Rk^-1;
    end
    for k=1:pf.Npts
        if TimeMeasBoth=='T' || TimeMeasBoth=='B'
            if isfield(pf,'fx')
                pf.particles(:,k) = feval(pf.fx, pf.particles(:,k), pf.tpara) + sQ*randn(pf.n,1);
            else
                pf.particles(:,k) = pf.Phikk_1*pf.particles(:,k) + sQ*randn(pf.n,1);
            end
        end
        if TimeMeasBoth=='M' || TimeMeasBoth=='B'
            if isfield(pf,'hx')
                v = Zk - feval(pf.hx, pf.particles(:,k), pf.tpara)';
            else
                v = Zk - pf.Hk*pf.particles(:,k);
            end
            vRv(k) = v'*invR*v;
        end
    end
    if TimeMeasBoth=='M' || TimeMeasBoth=='B'
%         [vRv, idx] = sort(vRv);  pf.particles = pf.particles(:,idx);
        weight = exp(-vRv/2);%/sqrt((2*pi)^pf.m*det(pf.Rk)); % Normal distribution
        cumw = cumsum(weight+0.01/pf.Npts);  cumw = cumw/cumw(end);
%         idx = interp1(cumw, (1:pf.Npts)', linspace(cumw(1),cumw(end),pf.Npts+10), 'nearest');  % resampling index
        idx = interp1(cumw, (1:pf.Npts)', cumw(1)+(cumw(end)-cumw(1))*rand(pf.Npts+10,1), 'nearest');  % resampling index
        pf.particles = pf.particles(:,idx(3:pf.Npts+2)); % myfig,hist(pf.particles(2,:));
    end
    pf.xk = mean(pf.particles,2);
    xk = pf.particles - repmat(pf.xk,1,pf.Npts);
	pf.Pxk = diag(sum(xk.^2,2)/pf.Npts);
%     for k=1:pf.n
%         pf.particles(k,:) = pf.xk(k)+sqrt(pf.Pxk(k,k))*randn(1,pf.Npts);
%     end
    if nargout==2  % if using full matrix Pxk
        pf.Pxk = zeros(pf.n);
        for k=1:pf.Npts
            pf.Pxk = pf.Pxk + xk(:,k)*xk(:,k)';
        end
        pf.Pxk = pf.Pxk/pf.Npts;
        Pxk = pf.Pxk;
    end
    