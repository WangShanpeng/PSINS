function bdpoint(app)
% Get point (lon,lat) from Baidu map.
%
% Prototype: bdpoint(app)
% Input: app - Windows application program case, msedge/chrome/firefox etc.
% Output: N/A
%
% See also  pos2bd, poscu.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 29/06/2024
    if nargin<1, app=1; end
    if app==1
        system('start msedge.exe https://api.map.baidu.com/lbsapi/getpoint/index.html');
    elseif app==2
        system('start chrome.exe https://api.map.baidu.com/lbsapi/getpoint/index.html');
    elseif app==3
        system('start firefox.exe https://api.map.baidu.com/lbsapi/getpoint/index.html');
    end