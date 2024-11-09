function Ft = oetm(ins)
% Open-loop SINS Error Transition Matrix.
%
% Prototype: Ft = oetm(ins)
% Input: ins - SINS structrue array
% Output: Ft - 15x15 error transition matrix
%
% See also  etm, insupdate.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 02/10/2024
    O33 = zeros(3);
	%%    fi               dvn       dpos   eb      db
	Ft = [-askew(ins.eth.wnie) O33        O33    -ins.Cnb  O33 
           askew(ins.fn)       O33        O33     O33      ins.Cnb 
           O33                 ins.Mpv    O33     O33      O33
           zeros(6,9)                             diag(-1./[ins.tauG;ins.tauA]) ];
