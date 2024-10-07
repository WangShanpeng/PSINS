function [pos, accu] = posbias(pos,b)
% Delete/add pos(lat,lon) bias, only applied for pos within China(73~135E¡¢3~54N).
%
% Prototype:  pos = posbias(pos,b)
% Inputs: pos - input pos data
%         b - latitude column from the last column (end-3), default 3
% Outputs: pos - output pos data
%          accu - float32 accuracy estimate
%
% See also  delbias, posdelol.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 02/08/2024
global glv
    if nargin<2, b=3; end
    idx = abs(pos(:,end-b+1))>3*glv.deg;  % non zero longitude
    if pos(idx(1),end-b+1)<70*glv.deg
        posf = pos(1,end-b:end-b+2)';
        pos(idx,end-b)  =pos(idx,end-b)  +30*glv.deg;
        pos(idx,end-b+1)=pos(idx,end-b+1)+110*glv.deg; 
    else
        pos(idx,end-b)  =pos(idx,end-b)  -30*glv.deg;
        pos(idx,end-b+1)=pos(idx,end-b+1)-110*glv.deg; 
        posf = pos(1,end-b:end-b+2)';
    end
    if nargout==2  % float32\single accuracy estimate
        lat=posf(1)+(0:0.01:1)/glv.Re;
        lon=posf(2)+(0:0.01:1)/glv.Re;
        hgt=posf(3)+(0:0.0001:0.01);
        accu=[max(abs(double(single(lat))-lat))*glv.Re; 
              max(abs(double(single(lon))-lon))*glv.Re;
              max(abs(double(single(hgt))-hgt))];
    end