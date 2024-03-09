function tf = tfplot(tf, flag)
% MINS/SINS transfer align output plot for C++ class 'CAligntf':
% void CAligntf::operator<<(CFileRdWt &f)
% {
% 	f<<sins.att<<sins.vn<<mu<<sins.eb<<sins.db  // 1-15
% 		<<lvMINS<<dtMINSdelay <<kftk; // 16-20
% }
%
% Prototype: tf = tfplot(tf, flag)
% Inputs: tf - MINS/SINS data array.
%         flag - plot flag
%          
% See also  igplot, igoplot, igkfplot, igload, odpplot, imuplot, gpsplot, dataplot.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/04/2023
global glv
    if nargin<2, flag=1; end
    if ischar(tf),  % tfplot(tf_fname);
        tf=binfile(tf,20);
        if flag~=1
            tf=adddt(tf,-flag);
        end
    end
    
    if flag==1
        myfig;
        subplot(421), plot(tf(:,end), tf(:,1:2)/glv.deg); xygo('pr');
        subplot(422), plot(tf(:,end), tf(:,3)/glv.deg); xygo('y');
        subplot(423), plot(tf(:,end), tf(:,4:6)); xygo('V');
        subplot(424), plot(tf(:,end), tf(:,7:9)/glv.min); xygo('mu');
        subplot(425), plot(tf(:,end), tf(:,10:12)/glv.dph); xygo('eb');
        subplot(426), plot(tf(:,end), tf(:,13:15)/glv.ug); xygo('db');
        subplot(427), plot(tf(:,end), tf(:,16:18)); xygo('L');
        subplot(428), plot(tf(:,end), tf(:,19)*1000); xygo('dT');
    end

