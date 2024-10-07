function [data, skip] = binfrmfile(binfile, header, fmt, scale, nfrm, littleEndian)
% Convert bin-byte-frame format file to double data.
%
% Prototype: [data, skip] = binfrmfile(binfile, header, fmt, scale, nfrm, littleEndian)
% Input: binfile - input bin file name
%        header - frame header, i.e. 'AA55' ect.
%        fmt - frame format, for example: 'u16 i24*6 n464'
%        scale - scale factor for each column
%        nfrm - n-frame to read
%        littleEndian - =1 for little-endian, =0 for big-endian
% Outputs: data - output data array
%          skip - skip bytes before header
%
% Example
%     glvs;
%     ts = 1/200;
%     % hexbinfile('0609.txt');
%     fmt = 'u16 u8 i24i24i24 i16 u8 u8';
%     s = [1 1 [1 1 1]/17920 1/256 1 1];
%     [data, skip] = binfrmfile('0609.bin', 'A5CC', fmt, s, [3,4]*3600/ts);
%     t = cnt2t(data(:,end-1),ts);  myfig, plot(t, data(:,6));  
%     avar(data(:,5)*3600, ts);
%
% See also  checkfrmfile, byte2num, hexbinfile, binfile, matbinfile.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 11/06/2024 
    if nargin<6, littleEndian=1; end
    if nargin<5, nfrm=inf; end
    if nargin<4, scale=1; end
    star=find(fmt=='*');  % make frame
    for k=length(star):-1:1
        m=sscanf(fmt(star(k)+1:end),'%d')-1;
        if m<10, fmt=[fmt(1:star(k)-1), repmat(fmt(star(k)-3:star(k)-1),1,m), fmt(star(k)+2:end)];
        else     fmt=[fmt(1:star(k)-1), repmat(fmt(star(k)-3:star(k)-1),1,m), fmt(star(k)+3:end)];  end
    end
    fmt(end+1) = 'n';  idx = find(fmt>'9'); typ=lower(fmt(idx));
    for k=1:length(typ)-1, len(k) = str2num(fmt(idx(k)+1:idx(k+1)-1)); end
    typ(end)=[]; len=len/8; k2=cumsum(len); k1=[0,k2(1:end-1)]+1;
    if length(scale)==1, scale=repmat(scale,1,length(typ)); end
    %%
    fid = fopen(binfile,'r');
    h1 = hex2dec(header(1:2)); h2 = hex2dec(header(3:4));  % find the header
    hd = fread(fid, 2, 'uint8');  skip=0;
    while 1
        if h1==hd(1) && h2==hd(2), fseek(fid,-2,'cof'); break; end
        hd(1)=hd(2); hd(2)=fread(fid, 1, 'uint8');  skip=skip+1;
    end    
    if length(nfrm)==2
        fseek(fid, k2(end)*nfrm(1), 'cof');  % skip nfrm(1), then read nfrm(2)
        dd = fread(fid, [k2(end), nfrm(2)], 'uint8=>uint8')';
    else
        dd = fread(fid, [k2(end), nfrm], 'uint8=>uint8')';
    end
    for k=1:length(k1)
%         if littleEndian==0, dd(:,k1(k):k2(k))=fliplr(dd(:,k1(k):k2(k))); end
%         if typ(k)=='i' && len(k)==1,
%             data(:,k)=double(typecast(dd(:,k1(k)),'int8'));
%         elseif typ(k)=='i' && len(k)==2,
%             u16 = uint16(dd(:,k1(k)))+uint16(dd(:,k1(k)+1))*256;
%             data(:,k)=double(typecast(u16,'int16'));
%         elseif typ(k)=='i' && len(k)==3,
%             u32 = uint32(dd(:,k1(k)))*256+uint32(dd(:,k1(k)+1))*256^2+uint32(dd(:,k1(k)+2))*256^3;
%             data(:,k)=double(typecast(u32,'int32')/256);
%         elseif typ(k)=='i' && len(k)==4,
%             u32 = uint32(dd(:,k1(k)))+uint32(dd(:,k1(k)+1))*256+uint32(dd(:,k1(k)+2))*256^2+uint32(dd(:,k1(k)+3))*256^3;
%             data(:,k)=double(typecast(u32,'int32'));
%         elseif typ(k)=='u' && len(k)==1,
%             data(:,k)=double(dd(:,k1(k)));
%         elseif typ(k)=='u' && len(k)==2,  
%             data(:,k)=double(uint16(dd(:,k1(k)))+uint16(dd(:,k1(k)+1))*256);
%         elseif typ(k)=='u' && len(k)==3,
%             u32 = uint32(dd(:,k1(k)))+uint32(dd(:,k1(k)+1))*256+uint32(dd(:,k1(k)+2))*256^2;
%             data(:,k)=double(u32);
%         elseif typ(k)=='u' && len(k)==4,
%             u32 = uint32(dd(:,k1(k)))+uint32(dd(:,k1(k)+1))*256+uint32(dd(:,k1(k)+2))*256^2+uint32(dd(:,k1(k)+3))*256^3;
%             data(:,k)=double(u32);
%         elseif typ(k)=='f' && len(k)==4,
%             u32 = uint32(dd(:,k1(k)))+uint32(dd(:,k1(k)+1))*256+uint32(dd(:,k1(k)+2))*256^2+uint32(dd(:,k1(k)+3))*256^3;
%             data(:,k)=double(typecast(u32,'single'));
%         elseif typ(k)=='f' && len(k)==8,
%             u64 = uint64(dd(:,k1(k)))+      uint64(dd(:,k1(k)+1))*256+  uint64(dd(:,k1(k)+2))*256^2+uint64(dd(:,k1(k)+3))*256^3+ ...
%                   uint64(dd(:,k1(k)))*256^4+uint64(dd(:,k1(k)+1))*256^5+uint64(dd(:,k1(k)+2))*256^6+uint64(dd(:,k1(k)+3))*256^7;
%             data(:,k)=double(typecast(u64,'double'));
%         elseif typ(k)=='n',
%             data(:,k)=zeros(size(dd(:,1)));
%         end
        data(:,k) = byte2num(dd(:,k1(k):k2(k)), typ(k), littleEndian);
        data(:,k) = data(:,k)*scale(k);
    end
    fclose(fid);
    