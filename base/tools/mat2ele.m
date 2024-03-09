function varargout = mat2ele(m)
% Decompose matrix to element.
%
% Prototype: varargout = mat2ele(m)
% Input: m - input matrix
% Outputs: varargout - output element
%
% Example
%   [a11,a12,a21,a22] = mat2ele([1,2;3,4]);
%
% See also  setvals.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 03/09/2023
    [k1,k2]=size(m);
    k = 1;  varargout{k1*k2} = 0;
    for k11=1:k1
        for k22=1:k2
            varargout{k} = m(k11,k22); k = k+1;
        end
    end