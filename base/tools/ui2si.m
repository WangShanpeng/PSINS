function si = ui2si(ui, bytes)
% trans unsinged interger to signed one
% Example
%   x=0:255; s=[]; u=[]; for k=1:length(x), s(k)=ui2si(uint8(x(k))); u(k)=si2ui(s(k)); end
% See also si2ui.
    if ui>=65536,
        if ui>2147483648, si=int32(ui-2147483648)-2147483648;
        else, si=int32(ui); end
    elseif ui>=256
        if ui>32768, si=int16(ui-32768)-32768;
        else, si=int16(ui); end
    elseif ui>127, si=int8(ui-128)-128;
    else, si=int8(ui); end
    if nargin>1
        switch bytes
            case 1, si=int8(si);
            case 2, si=int16(si);
            case 32, si=int32(si);
        end
    end