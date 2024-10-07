function num = byte2num(bytes, type, littleEndian)
% Convert bytes to double number.
%
% Prototype: num = byte2num(bytes, type, littleEndian)
% Input: bytes - uint8 type column bytes input
%        type - int, uint, or float data output type.
%        littleEndian - =1 for little-endian, =0 for big-endian
% Outputs: num - double output data array
%
% See also  binfrmfile, hexbinfile, dec2num, typecast.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 11/06/2024 
    if nargin<3, littleEndian=1; end
    if nargin<2, type='i'; end  % default for int type
    if length(type)>1  % more than 1 comumn output, type='f32' etc.
        n = sscanf(type(2:end),'%d')/8;  % n: in bytes
        kk=1;
        for k=1:n:size(bytes,2)
            if kk*n>size(bytes,2), break; end
            num(:,kk) = byte2num(bytes(:,k:kk*n), type(1), littleEndian);  kk=kk+1;
        end
        return;
    end
    if littleEndian==0, bytes=fliplr(bytes); end
    [m, n] = size(bytes);
    if n==8, type='f'; end % default 8-bytes for doulbe type
    if type=='i'
        if n==1
            num=double(typecast(bytes,'int8'));
        elseif n==2
            u16 = uint16(bytes(:,1))+uint16(bytes(:,2))*256;
            num = double(typecast(u16,'int16'));
        elseif n==3
            u32 = uint32(bytes(:,1))*256+uint32(bytes(:,2))*256^2+uint32(bytes(:,3))*256^3;
            num = double(typecast(u32,'int32')/256);
        elseif n==4,
            u32 = uint32(bytes(:,1))+uint32(bytes(:,2))*256+uint32(bytes(:,3))*256^2+uint32(bytes(:,4))*256^3;
            num = double(typecast(u32,'int32'));
        end
    elseif type=='u'
        if n==1
            num = double(bytes);
        elseif n==2
            num = double(uint16(bytes(:,1))+uint16(bytes(:,2))*256);
        elseif n==3
            u32 = uint32(bytes(:,1))+uint32(bytes(:,2))*256+uint32(bytes(:,3))*256^2;
            num = double(u32);
        elseif n==4
            u32 = uint32(bytes(:,1))+uint32(bytes(:,2))*256+uint32(bytes(:,3))*256^2+uint32(bytes(:,4))*256^3;
            num = double(u32);
        end
    elseif type=='f'
        if n==4
            u32 = uint32(bytes(:,1))+uint32(bytes(:,2))*256+uint32(bytes(:,3))*256^2+uint32(bytes(:,4))*256^3;
            num = double(typecast(u32,'single'));
        elseif n==8
            u64 = uint64(bytes(:,1))+      uint64(bytes(:,2))*256+  uint64(bytes(:,3))*256^2+uint64(bytes(:,4))*256^3+ ...
                  uint64(bytes(:,5))*256^4+uint64(bytes(:,6))*256^5+uint64(bytes(:,7))*256^6+uint64(bytes(:,8))*256^7;
            num = double(typecast(u64,'double'));
        end
	else
        num = zeros(m,1);
    end

    