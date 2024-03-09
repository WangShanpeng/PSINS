function plotline(x1, y1, x2, y2)
% Plot lines.
%
% Prototype: plotline(x1, y1, x2, y2)
% Inputs: x1,y1 - start points, default (0,0)
%         x2,y2 - end points
%
% Examples
%    myfig, plotline([1,2], [2;3], 5, [5,6]); plotline(5, [15,16]);
% 
% See also  plotn.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 29/01/2023
    if nargin<3, x2=x1; y2=y1; x1=x1*0; y1=y1*0; end
    len = length(y1);
    if length(x1)==1, x1=repmat(x1,len,1); end
    if length(x2)==1, x2=repmat(x2,len,1); end
    nextlinestyle(-1);
    for k=1:len
        hold on; plot([x1(k);x2(k)],[y1(k);y2(k)], nextlinestyle(1));
    end
    grid on;