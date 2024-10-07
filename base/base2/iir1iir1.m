function y = iir1iir1(x, afa)
% Zero-phase forward and reverse IIR1 filtering, yk = (1-afa)*yk_1 + afa*xk.
%
% Prototype: y = iir1iir1(x, afa)
% Inputs: x - input data
%         afa - correlation time
% Outputs: y - output data
%
% Example:
%    x = randn(100,2)+1; figure, plot([x, iir1iir1(x)]); grid on;
%
% See also  filtfilt, iir1.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 18/06/2024
    if nargin<2, afa=0.1; end
    y = iir1(x, afa);
    y = flipud(y);
    y = iir1(y, afa);
    y = flipud(y);
