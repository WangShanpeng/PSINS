function mag = magsimu(att,  declination, inclination, amplitude, noise, ifplot)
% Earth's magnetic simulation.
%
% Prototype: mag = magsimu(att,  declination, inclination, amplitude, noise, ifplot)
% Inputs: att - the body attitude
%         declination - magnetic declination in counter-clockwise positive convension
%         inclination - magnetic inclination in header-up positive convension
%         amplitude - magnetic inclination in Gauss
%         noise - noise in Gauss
%         ifplot - plot results after simulation
% Output: mag - Earth's magnetic in body frame.
%
% Example:
%   glvs
%   att = zeros(100,4); att(:,4) = (1:100)';
%   mag = magsimu(att, 10*glv.deg, 30*glv.deg, 500, 10, 1);
%          
% See also  trjsimu, odsimu, gpssimu.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 21/01/2021
    Cnm = a2mat([inclination;0;declination]);  % like Cnb
    Cmn = Cnm';
    mag = att(:,[1:3,end]);
    for k=1:length(mag)
        Cmb = Cmn*a2mat(att(k,1:3)');
        mag(k,1:3) = Cmb(2,:);  % Cbm*[0;1;0];
    end
    mag(:,1:3) = mag(:,1:3)*amplitude + randn(length(mag),3)*noise;
	if ifplot==1,  
        magplot(mag);
	end

