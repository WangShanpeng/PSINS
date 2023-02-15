function avperr = inserrb(avperr, avp, isfig)
% trans avp error to body-frame.   bad!
%
% Prototype: inserrplot(err, ptype)
% Inputs: err - may be [phi], [phi,dvn], [phi,dvn,dpos],
%                      [phi,dvn,dpos,eb,db], etc.
%         ptype - plot type define
%          
% See also  insserrplot.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/12/2022
    if nargin<3, isfig=1; end
    eth = earth(avp(1,7:9)');
	Mpv=zeros(3); Mpv(4)=1/eth.RMh; Mpv(2)=1/eth.clRNh; Mpv(9)=1; Mpv1=(Mpv^-1)';
    att = interp1(avp(:,end), avp(:,1:3), avperr(:,end), 'nearest');
    n = size(avperr,2);
    for k=1:length(avperr)
        Cnb = a2mat(att(k,1:3)');
        avperr(k,end-3:end-1) = avperr(k,end-3:end-1)*Mpv*Cnb*Mpv1;
        if n>6
            avperr(k,end-6:end-4) = avperr(k,end-6:end-4)*Cnb;
        end
    end
    if isfig==1
        if n==4
            inserrplot(avperr,'p');
        elseif n==7
            inserrplot(avperr,'vp');
        elseif n==10
            inserrplot(avperr,'avp');
        end
    end
