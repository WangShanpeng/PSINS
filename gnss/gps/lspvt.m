function [pvt, vpENU, res] = lspvt(recPos0, satpv, obs)
% Calculate receiver's position using least square method.
%
% Prototype: [recPos, res] = lspvt(recPos0, satPosVel, obs)
% Inputs: recPos0 - initial receiver position [X, Y, Z, clock-bias in meter]
%         satpv - satellite positions /and velocities
%         obs - obs=[PR] or obs=[PR,PRDot], where PR is pseudo-range observations,
%               including satellite clock corrections, and PRDot is Doppler
%               observation. Make sure satellite num >= 4.
% Outputs: pvt - 16x1 vector=[posXYZ;dt; Perr; satNum; DOP; velXYZ;dtDot; Verr]
%          res - some other results, see the code.
%
% See also  satPosVel, lsVel, spp, DOP, maxtetra.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 14/10/2013, 30/06/2015
    satPos=satpv(:,1:3); PR=obs(:,1);
    rho = PR - recPos0(4);  % ?
    for iter = 1:20
        [rho, LOS, AzEl, pos, Cen] = rhoSatRec(satPos, recPos0, rho);
        el = AzEl(:,2); el(el<15*pi/180) = 0.01*pi/180;
        W = diag(sin(el).^2); % eye(length(el)); % weight matrix
        dPR = PR - rho - recPos0(4);
        A = [-LOS, ones(size(LOS(:,1)))];
        x = (A'*W*A)^-1*A'*W*dPR;
        recPos = recPos0 + x;
        if x'*x<0.001^2,  break;  end
        recPos0 = recPos;
    end
    err = sqrt(W)*(dPR - A*x);  errPos = sqrt(err'*err);  % LS residual error
    if size(obs,2)==2  % receiver velocity by Doppler observation
        satVel=satpv(:,4:6); PRDot=obs(:,2);
        [recVel, errVel] = lsVel(satVel, LOS, W, PRDot, rho);
    else
        [recVel, errVel] = setvals(zeros(4,1), 0);
    end
    pvt = [recPos;errPos;length(rho);DOP(A); recVel;errVel];
    if nargout>1
        vpENU = [Cen'*recVel(1:3);pos];
    end
    if nargout>2
        res.rho = rho;  res.LOS = LOS;  res.AzEl = AzEl;
        res.Cen = Cen;
        res.W = W;
    end
