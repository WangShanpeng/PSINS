function pos = poscu(strname)
% Geographic position in common use.
%
% Prototype: pos = poscu(strname)
% Input: strname=string name of institude NO.
% Output: pos=[lat; lon; hgt]
% 
% See also  posset.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 13/09/2024
    if isnumeric(strname)
        switch strname
            case 16,  pos = llh();
            case 203,  pos = llh();
        end
    else
        switch upper(strname)
            case 'NWPU',  pos = llh(34.0343100, 108.7754270, 450);
            case 'JZ',    pos = llh(34.1716655, 108.8435892, 431);
            case 'LD',    pos = llh();
        end
    end
