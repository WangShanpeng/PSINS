function dec = sbin2dec(sbin)
% See also bin2dec.
    dec = bin2dec(sbin);
    if sbin(1)=='1' % if negative
        dec = dec - 2^length(sbin);
    end