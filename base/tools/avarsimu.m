function [y, t] = avarsimu(NRKQ, M, ts, len, isfig)
% Gyro Allan noise simulation.
%
% Prototype: y = avarsimu(NRKQ, M, ts, len, isfig)
% Inputs: NRKQ - = [N,R,K,Q] parameters, where
%             N: angular random walk, in deg/h^{1/2}
%             R: rate ramp, in deg/h^2
%             N: angular rate random walk, in deg/h^{3/2}
%             Q: angular quantization(or algular Gauss white noise, 1-sigma), in arcsec
%         M - = [sigma1, tau1; sigma2, tau2; ...] 1st order Markov process, where
%             sigma: standard deviation of the process
%             tau: correlation time
%         ts - sampling interval
%         len - data length
%         isfig - figure flag
% Outputs: y - gyro Allan noise output, in deg/h
%         t - sampling time stamp
%
% Example: 
%     y = avarsimu([0.001,0.05,0.01,.01], [1, 1], 0.01, 1000000, 1);
%     [sigma, tau] = avar(y, 0.01, 2);
%     avarfit(sigma, tau);
%
% See also  avar, avarfit, markov1, quantiz.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 20/05/2021
global glv
    t = (1:len)'*ts;
    y = NRKQ(1)*glv.dpsh/sqrt(ts)*randn(len,1);  % N
    if length(NRKQ)>1,  % R
        y = y + NRKQ(2)*glv.dph2*t;
    end
    if length(NRKQ)>2,  % K
%         y = y + cumsum(NRKQ(3)*glv.dphpsh/ts*randn(len,1));
        y = y + cumsum(NRKQ(3)*glv.dphpsh*sqrt(ts)*randn(len,1));
    end
    for k=1:size(M,1)   % Markov
        y = y + markov1(M(k,1)*glv.dph, M(k,2), ts, len);
    end
    if length(NRKQ)>3,  % Q
        if NRKQ(4)>0
            Q = NRKQ(4)*glv.sec;
%             y = y+diff(randn(length(y)+1,1)*Q/ts);  % angular Gauss white noise, NRKQ(4) is the 1-sigma std
            y = quantiz(y*ts+10*Q, sqrt(12)*Q)/ts - 10*Q/ts;  % quantization noise
        end
    end
    y = y/glv.dph;
    if ~exist('isfig','var'), isfig=0; end
    if isfig
        myfig;
        plot(t, y); xygo('\itx \rm/ ( (\circ) / h )');
%         if exist('y0','var'), hold on; plot(t,y0/glv.dph,'r'); end
    end