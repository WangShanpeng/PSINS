function [mu, Cb00b1] = aa2muEx(Cnb0, Cnb1)
% Calculate the installation error angles(expressed in body-frame) 
% between att1 and att0, even for large misalignment (trans to small).
%
% Prototype: [mu, Cb00b1] = aa2muEx(Cnb0, Cnb1)
% Inputs: Cnb0 - DCM or Euler angle 0
%         Cnb1 - DCM or Euler angle 1
% Output: mu - installation error angles mu=[mux;muy;muz]
%
% See also  aa2mu, aa2phi, aa2phimu.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/09/2022
    if nargin==1, Cnb1=Cnb0; Cnb0=zeros(3,1); end
    if size(Cnb0,2)==1, Cnb0=a2mat(Cnb0); end
	if size(Cnb1,2)==1, Cnb1=a2mat(Cnb1); end
    Cb0b1 = Cnb0'*Cnb1;
    Cb00b1 = zeros(3);
    ii = 0;
    for k=1:9
        if Cb0b1(k)>0.71, Cb00b1(k)=1; ii=ii+1;
        elseif Cb0b1(k)<-0.71, Cb00b1(k)=-1; ii=ii+1;
        end
    end
	Cb0b1 = Cnb0'*Cnb1*Cb00b1';
    mu = zeros(3,1);
    if ii==3, mu = m2att(Cb0b1); end

    