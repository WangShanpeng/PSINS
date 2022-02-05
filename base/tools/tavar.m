function [sigma, tau] = tavar(y0, tau0)
% Calculate total Allan variance.
%
% Prototype: [sigma, tau] = oavar(y0, tau0)
% Inputs: y0 - data 
%         tau0 - sampling interval
% Outputs: sigma - Allan variance
%          tau - Allan variance correlated time
%
% Example: 
%     y = randn(1000,1) + 0.01*[1:1000]';
%     [sigma, tau] = tavar(y, 1);
%
% See also  avar, oavar.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/01/2022
    m = 100;
    m0 = mean(y0(1:m));
    m1 = mean(y0(end-m+1:end));
    y1 = flipud(-y0);
    [sigma, tau] = avar([y1+2*m0; y0; y1+2*m1], tau0, 1);
    
