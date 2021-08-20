function Cb1b2 = Cbbstr(b1str, b2str)
% DCM from b1-frame to b2-frame.
%
% Prototype: att1 = attrfu(att0, dirstr)
% Inputs: b1str,b2str - the orientation string abbreviations are:
%               'U': Upper; 'D': Down; 'R': Right; 'L': Left; 
%               'F': Forword; 'B': Back.
% Outputs: Cb1b2 - DCM C^b1_b2.
%
% Example:
%   Cb1b2 = Cbbstr('rfu', 'flu')
%
% See also  attrfu, imurfu.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 30/07/2021
    [~, Cnb1] = imurfu([], b1str);
    [~, Cnb2] = imurfu([], b2str);
    Cb1b2 = Cnb1'*Cnb2;