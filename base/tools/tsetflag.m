function flag = tsetflag(t, varargin)
% Set time flag between some intervals.
%
% Prototype: flag = tsetflag(t, t00,t01, t20,t21, ...)
% Inputs: t - time stamp
%         t00/t01,t20/t21,... - time interval between which to set flag 1 
%
% See also  sortt, tshift, tsyn.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 15/01/2021
    flag = false(size(t));
    n = (nargin-1)/2;
    for k=1:n
        t0=varargin{2*k-1}; t1=varargin{2*k};
        flag = flag | (t0<=t & t<t1);
    end
