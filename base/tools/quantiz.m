function yi = quantiz(x, q, q2)
% angular/velocity increment quantization.
%
% Prototype: yi = quantiz(x, q)
% Inputs: x - data source to be quantized (angular/velocity increment)
%         q, q2 - quantitative equivalent
% Output: yi - data output after quantization
%
% Example:
%     glvs; T = 1000; ts = 0.01; t = (0:ts:T)';
%     Theta = interp1(0:T, randn(T+1,1)*10*glv.sec, t, 'spline');
%     DTheta = diffs(Theta) + 1.0123*glv.sec;
%     DThetaQ = quantiz(DTheta, 1*glv.sec);  % Q=D/sqrt(12)=D/3.5
%     err = cumsum(DThetaQ)-cumsum(DTheta);
%     myfig, subplot(311), plot(t, [DTheta,DThetaQ]/glv.sec); xygo('\Delta\theta / \prime\prime');
%     subplot(312), plot(t, err/glv.sec); xygo('Q / \prime\prime');
%     subplot(325), hist(err/glv.sec, 20); subplot(326), hist(DThetaQ/glv.sec, 20); 
%
% See also  imuresample.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 07/11/2013, 16/06/2021
    if size(x,2)==7  % imu1 = quantiz(imu, qgyro, qacc);
        if nargin==3, q(2)=q2; end
        yi = x;
        yi(:,1:3) = quantiz(x(:,1:3), q(1));
        yi(:,4:6) = quantiz(x(:,4:6), q(2));
        return;
    end
%     x = floor(cumsum(x,1)/q);
    x = round(cumsum(x,1)/q);
    yi = diff([zeros(1,size(x,2)); x])*q;
