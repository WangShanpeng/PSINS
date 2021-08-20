function [data, badlines] = loadtxt(fname, sizemn, ncl, ncl0)
% Load txt data file by replacing non numeral character with blank space(' ') or character('0').
% For example, if a line in the 'file.txt' like this:
%    2016/09/03,19:48:49 	18бу12.59'	109бу41.21'	0бу52.44'
% the function call is
%    data = loadtxt('file.txt', [100,12], '/:бу''');
%
% Prototype: data = loadtxt(fname, sizemn, ncl, ncl0)
% Inputs: fname - txt file name to be saved 
%         sizemn - data size to load
%         ncl - non numeral character list replaced with ' '
%         ncl0 - non numeral character list replaced with '0'
% Outputs: data - data read from txt file
%          badlines - bad lines unsatisfied numeral line

% See also  txtfile.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 01/08/2020
    if ~exist('ncl0', 'var'), nons0=0; else, nons0=length(ncl0); end
    if nargin<3, ncl = sizemn; sizemn = 0; end  % [data, badlines] = loadtxt(fname, ncl)
    if length(sizemn)==1, sizemn = [1e6, sizemn]; end
    m = sizemn(1); n = sizemn(2);
	if n>0,  data = zeros(m,n);  end
    nons = length(ncl);
    kk = 1; badlines = 0;
    fid = fopen(fname);
    while (1)
        tline = fgetl(fid);
        if ~ischar(tline), break; end
        for k1=1:nons
            tline(strfind(tline, ncl(k1))) = ' ';
        end
        for k1=1:nons0
            tline(strfind(tline, ncl0(k1))) = '0';
        end
        num = str2num(tline);
        if n==0, n = length(num); data = zeros(m,n);  end  % make sure the 1st line is OK
        if length(num)==n
            data(kk,:) = num; kk = kk+1;
        else
            badlines = badlines + 1;
        end
        if kk>m, break; end
    end
    data(kk:end,:) = [];
    fclose(fid);

