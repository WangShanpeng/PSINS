function pos = llh(lat, lon, hgt)
% Geographic position = [latitude; logititude; height] setting.
%
% Prototype: pos = llh(lat, lon, hgt)
% Input: pos=[lat; lon; height], where lat and lon are in arcdeg,
%             & height is in m.
% Output: pos=[lat*arcdeg; lon*arcdeg; hgt]
% 
% See also  posset, pry, avpset.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/08/2024
global glv
    if nargin<2, lon=0; hgt=0;                  % pos=llh(lat);
    elseif nargin<3,
        if abs(lon)<=180, hgt=0;                 % pos=llh(lat,lon);  
        else,             hgt=lon; lon=0;  end   % llh(lat,hgt);
    end;
    pos = [lat*glv.deg; lon*glv.deg; hgt];