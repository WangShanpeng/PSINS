function [avp, varargout] = insload(fname, clms, t0)
% SINS output bin file load for C++ class 'CSINS':
%   fins << kf.sins << var1 << var2 << ...;
%
% Prototype: [avp, varargout] = insload(fname, clms, t0)
% Inputs: fname - SINS bin file name
%         clms - data columns for varargout
%         t0 - start time
% Outputs: xxx
%          
% See also  igload, igoload, insload.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/01/2023
    n = sum(clms)+16;
    avp = binfile(fname, n);
    if nargin>2, avp(:,16) = avp(:,16)-t0; end
    clms = [16; cumsum(clms(:))+16];
    for k=1:length(clms)-1
        varargout{k} = [avp(:,(clms(k)+1):clms(k+1)), avp(:,16)];
    end
    avp = avp(:,1:16);
