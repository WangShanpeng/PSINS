function [att, vn, pos] = avp2avp(att, vn, pos)
% avp decomposition or reverse.
%
% Prototype: [att, vn, pos] = avp2avp(att, vn, pos)
% Inputs: att, vn, pos - input avp
% Outputs: avp - att, vn, pos - output avp
%
% See also  avpcvt, avpset.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 22/08/2024
	if nargin>1  % composition: avp = avp2avp(att, vn, pos)
        if nargin==2, pos=vn; vn=[0;0;0]; end  % avp = avp2avp(att, pos)
        if length(att)==4; att=q2att(att);  % avp = avp2avp(qnb, vn, pos)
        elseif numel(att)==9, att=m2att(att);  end  % avp = avp2avp(Cnb, vn, pos)
        if length(pos)==1, pos=[pos;0;0];  % avp = avp2avp(att, lat)
        elseif length(pos)==2, pos=[pos;0]; end  % avp = avp2avp(att, [lat;lon])
        att = [att;vn;pos];
    else  %  decomposition: [att; vn; pos] = avp2avp(avp)
        att = avp(1:3);
        switch length(avp)
            case 10, att = q2att(avp(1:4)); vn = avp(5:7); pos = avp(8:10);
            case 9,  vn = avp(4:6); pos = avp(7:9);
            case 8,  vn = avp(4:6); pos = [avp(7:8);0];
            case 7,  vn = avp(4:6); pos = [avp(7);0;0];
            case 6,  vn = [0;0;0];  pos = avp(4:6);
            case 5,  vn = [0;0;0];  pos = [avp(4:5);0];
            case 4,  vn = [0;0;0];  pos = [avp(4);0;0];
        end
    end
    