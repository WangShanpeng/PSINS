function trp = saastamoinen(pos, el)
% Compute tropospheric delay by standard atmosphere and saastamoinen model.
% Ref RTKLIB 2.4.2 'tropmodel'.
%
% See also  bdKlobuchar.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 31/07/2015
    if size(pos,2)==1, pos = pos'; end
    
    humi = 0.7;
    temp0 = 15.0; % temparature at sea level
       
    % standard atmosphere
    hgt=pos(:,3); hgt(hgt<0)=0; hgt(hgt>11000)=11000;  % Ref. ISA(国际标准大气)
    pres = 1013.25 * (1.0-2.2557E-5*hgt).^5.2568;
    temp = temp0 - 6.5e-3*hgt + 273.16;
    e = 6.108*humi*exp((17.15*temp-4684.0)./(temp-38.45));
    
    % saastamoninen model
    z = pi/2.0 - el;
    trph = 0.0022768*pres./(1.0-0.00266*cos(2.0*pos(:,1))-0.00028*hgt/1e3)./cos(z);
    trpw = 0.002277*(1255.0./temp+0.05).*e./cos(z);
    trp = trph + trpw;

