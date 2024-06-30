function str = nextlinestyle(k)
% Get line-style string for plot.
%
% str = nextlinestyle(k)
% Input: k - next style index to add.
%            -1 for reset, 0 for no-change, 1 for next style
% Output: str - style string
%
% See also  labeldef, xygo.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/08/2022
    global glv_linestyle
    if nargin<1, k=1; end
    if k==-1, glv_linestyle=[]; k=0; end  % reset
    if isempty(glv_linestyle)
        glv_linestyle.style = { '-b'; '-g'; '-r'; '-c'; '-.b'; '-.g'; '-.r'; '-.c'; ':b'; ':g'; ':r'; ':c'; '--b'; '--g'; '--r'; '--c' };
%         glv_linestyle.style = { '-bo'; '-g^'; '-rv'; '-ms'; '-.bo'; '-.g^'; '-.rv'; '-.ms'; ':bo'; ':g^'; ':rv'; ':ms'; '--bo'; '--g^'; '--rv'; '--ms' };
        glv_linestyle.len = length(glv_linestyle.style);
        glv_linestyle.kk = 0;
    end
    glv_linestyle.kk = mod(glv_linestyle.kk+(k-1),glv_linestyle.len)+1;
    str = glv_linestyle.style{glv_linestyle.kk};
