function [sigma, tau] = oavar(y0, tau0)
% Calculate overlapping Allan variance.
%
% Prototype: [sigma, tau] = oavar(y0, tau0)
% Inputs: y0 - data 
%         tau0 - sampling interval
% Outputs: sigma - Allan variance
%          tau - Allan variance correlated time
%
% Example: 
%     y = randn(1000,1) + 0.01*[1:1000]';
%     [sigma, tau] = oavar(y, 1);  figure, loglog(tau, sigma); grid on
%
% See also  avar.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/01/2022
    if nargin<2, tau0=1; end
    M = length(y0); M2 = floor(M/2);
    sigma = zeros(M2,1);
    for m = 1:M2  % GuoHairong (4.5)  ?
        dy2 = 0;
        for j=1:(M-2*m+1)
            dy = y0(j+1:j+m)-y0(j:j+m-1);
            dy2 = dy2 + dy'*dy;
        end
        sigma(m) = sqrt(dy2/(2*m^2*(M-2*m+1)));
    end
    tau = (1:M2)'*tau0;
    return;
    
%     if nargin<2, tau0=1; end
%     N = length(y0);
%     y = y0; NL = N;
%     for k = 1:log2(N)
%         dy = y(2:NL-1)-y(1:NL-2);
%         sigma1 = sqrt(1/(2*(NL-2))*(dy'*dy)); % diff&std
%         dy = y(3:NL)-y(2:NL-1);
%         sigma2 = sqrt(1/(2*(NL-2))*(dy'*dy)); % diff&std
%         sigma(k,1) = (sigma1+sigma2)/2;  % mean avar
%         tau(k,1) = 2^(k-1)*tau0;      % correlated time
%         NL = floor(NL/2);
%         if NL<3
%             break;
%         end
%         y = 1/2*(y(1:2:2*NL) + y(2:2:2*NL));  % mean & half data length
%     end
%     return;

% %
%     if nargin<2, tau0=1; end
%     m = 8;
%     for k=1:m
%         k1 = 2^k-1;  k2 = 2^m - k1 - 1;
%         [sigma(:,k), tau] = avar(y0(k1:end-k2,1), tau0);
%     end
%     sigma = mean(sigma,2);
%     return;
% %    
%     if nargin<2, tau0=1; end
%     len = length(y0);
%     sigma = zeros(len-1,1);
%     for k=1:len-2
%         dy = (y0(k+1:len,1) - y0(1:len-k,1)) / sqrt(k);  % mean & diff
%         sigma(k) = sqrt( (dy'*dy) / (2*(len-k)) );
%     end
%     tau = (1:len-1)'*tau0;
