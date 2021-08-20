function wbie = wierfu(dirstr, lat, att0)
% Calculate the earth angular rate 'wie' in SIMU frame, where SIMU is
% put in 'dirstr' direction.
%
% Prototype: wie = wieenu(dirstr, lat, yaw)
% Inputs: dirstr - SIMU X-Y-Z orientations including three characters, 
%               the orientation abbreviations are:
%               'U': Upper; 'D': Down; 'R': Right; 'L': Left; 
%               'F': Forword; 'B': Back; 'E': East; 'W': West; 'N': North;
%               'S': South.
%         lat - latitude
%         att0 - initial attitude direction
% Output: wie - The earth angular rate.
%
% Example:
%         wbie = wierfu('ENU', 30*pi/180, 10*glv.deg)/glv.dph
%
% See also  imurfu, imurot, imuresample, insupdate, trjsimu.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 02/12/2020
    if nargin<3, att0=[0;0;0]; end
    if length(att0)==1, att0=[0;0;att0]; end
    eth = earth([lat(1);0;0]);
    wbie = a2mat(att0)'*eth.wnie;
    [~, Dir] = imurfu([], dirstr);
    wbie = Dir'*wbie;

