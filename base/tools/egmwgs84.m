function [gn, gLh] = egmwgs84(lat, hgt)
    if nargin==1, hgt=lat(end); lat = lat(1)*pi/180; end
    lat = lat*pi/180;
    C = zeros(11);  S = C;
    C(1:2:11,1) = [1;                       -0.108262982131*10^-2/sqrt(5); % WGS-84 coefficients
            0.237091120053*10^-5/sqrt(9);   -0.608346498882*10^-8/sqrt(13); 
            0.142681087920*10^-10/sqrt(17); -0.121439275882*10^-13/sqrt(21) ];
    GM = 3.986004415e14;  Re = 6.378136998405e6;  wie = 7.2921151467e-5;  % WGS-84 model
    ff = 1/298.257223563;
    gn = egm(GM, Re, wie, ff, C, S, lat, 0, hgt);
    gLh = -gn(3);
