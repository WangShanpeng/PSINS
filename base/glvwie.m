function glvwie(flag)
% Set zero or restore glv.wie.
% See also  glvs.
% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 12/05/2023
    global glv
    if nargin<1, flag=1; end
    if flag==0; glv.wie=0; else, glv.wie=7.2921151467e-5;  end
