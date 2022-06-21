function dop = DOP(A)
% Calculate GPS positioning DOP values.
%
% Input: A - = [-losEi, -losNi, -losUi, 1]
% See also  lspvt, maxtetra.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 30/08/2013, 30/06/2015, 22/10/2021
    for k=1:size(A,1), A(k,1:3) = A(k,1:3)/sqrt(A(k,1:3)*A(k,1:3)'); end
    if size(A,2)==3
        Q = inv(A'*A);
        dop = sqrt([...
            Q(1,1) + Q(2,2) + Q(3,3);                % PDOP
            Q(1,1) + Q(2,2);                         % HDOP
            Q(3,3);                                  % VDOP
            ]);
        return;
    end
    Q = inv(A'*A);
    dop = sqrt([...
        Q(1,1) + Q(2,2) + Q(3,3) + Q(4,4);       % GDOP
        Q(1,1) + Q(2,2) + Q(3,3);                % PDOP
        Q(1,1) + Q(2,2);                         % HDOP
        Q(3,3);                                  % VDOP
        Q(4,4);                                  % TDOP
        ]);
        
