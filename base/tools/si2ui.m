function ui = si2ui(si, bytes)
% trans singed interger to unsigned one
% Example
%   x=uint16(0:1000); s=[]; u=[]; for k=1:length(x), s(k)=ui2si(x(k),2); u(k)=si2ui(s(k),2); end
%   myfig, plot(x, s);
% See also ui2si.
    if si<-2147483648, ui=uint32(si+2147483648)+2147483648;
    elseif si<-128, ui=uint16(si+32768)+32768;
    elseif si<0, ui=uint8(si+128)+128;
    else, ui=uint8(si); end
    if nargin>1
        switch bytes
            case 1, ui=int8(ui);
            case 2, ui=int16(ui);
            case 32, ui=int32(ui);
        end
    end