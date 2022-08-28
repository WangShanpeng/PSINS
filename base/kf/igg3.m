function w = igg3(err, k0, k1)
% 'Institute of Geodesy & Geophysics' picewise method to calculate weight.
% Ref. IGG抗差估计在高程拟合中的应用研究_李广来,2021
%
% Prototype: gamma = igg3(err, k0, k1)
% Inputs: err - normalized measurement error
%         k0, k1 - picewise points 
% Outputs: w - weight
%
% Example:
%    figure, err=-10:0.1:10; plot(err,igg3(err,3,8)); grid on
%
% See also  igg1.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 20/06/2022
    if nargin==1, k0=2.0; k1=6.0; end  % w = igg3(err)
    if nargin==2, k1=k0(2); k0=k0(1); end  % w = igg3(err, k0k1)
    if k1<2*k0, k1=2*k0; end
    err = abs(err);  w = err;
    for k=1:length(err)
        if err(k)<=k0,    w(k)=1;
        elseif err(k)>k1, w(k)=0;
        else,             w(k)=k0/err(k) * ((k1-err(k))/(k1-k0))^2;  end
    end
    