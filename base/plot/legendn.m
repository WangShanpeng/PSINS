function legendn(n)
% Legend of specific characteristic.
%
% Prototype: legendn(n)
% Input: n - n lines
%
% See also  mylegend.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 26/11/2023
    for k=1:n
        str{k} = sprintf('%d',k);
    end
	legend(str);
    
