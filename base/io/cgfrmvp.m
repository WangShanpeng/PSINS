function dt = cgfrmvp(cgfile, vp, vperr)
% '*.cg' format file from vel_pos array 
    if nargin<3, vperr = [1;1;1; 0.1;0.1;0.1]; end
    vn = single(vp(:,1:3));
    vperr = single(vperr);
    sg0 = single(0.0);
    fid = fopen(cgfile,'r');
    fid1 = fopen([cgfile(1:end-3),'_simu.fg'],'w');
    [fh,n,t1,t2,pos0,gu1] = cgheader(fid);  cgcopy(fid1, fh);
    ts = diff(vp(1:2,end));  tt = zeros(n,2);
    dt = t1-ts;
    timebar(1, n, 'cg-from-vp porcessing,');
    for k=1:n
        gu = cgrecord(fid);
        if k<=length(vp),
%             gu.dGpsTime = fix(t1+(k-1)*ts);  % gu.dCorrTime = gu.dGpsTime;
            gu.dLatitude = vp(k,4);  gu.dLongitude = vp(k,5);  gu.dEllHt = vp(k,6);        
            gu.fLLVE = vn(k,1);    gu.fLLVN = vn(k,2);    gu.fLLVZ = vn(k,3);
        else, 
            gu.dLatitude = vp(end,4);  gu.dLongitude = vp(end,5);  gu.dEllHt = vp(end,6);        
            gu.fLLVE = vn(end,1);    gu.fLLVN = vn(end,2);    gu.fLLVZ = vn(end,3);
        end
        gu.usRecFlag = gu1.usRecFlag; gu.ucQuality = gu1.ucQuality; gu.ucNumGPS = gu1.ucNumGPS;
        gu.fLLAE = sg0;    gu.fLLAN = sg0;    gu.fLLAZ = sg0;
        gu.fSDE = vperr(1);    gu.fSDN = vperr(2);    gu.fSDZ = vperr(3);
        gu.fCEN = sg0;    gu.fCNZ = sg0;    gu.fCEZ = sg0;
        gu.fSDVE = vperr(4);    gu.fSDVN = vperr(5);    gu.fSDVZ = vperr(6);
        gu.fCVEN = sg0;    gu.fCVNZ = sg0;    gu.fCVEZ = sg0;
        cgcopy(fid1, gu);
        tt(k,:) = [gu.dGpsTime, gu.dCorrTime];
        timebar;
    end
    cgcopy(fid1, fid);
    fclose(fid); fclose(fid1);
    
function [fh, numrec, t1, t2, pos0, gu1] = cgheader(fid)
    fh.szHdrstr = fread(fid, [1,8], '*char');  % File Header
    fh.dFileVersion = fread(fid, 1, 'double');
    fh.ucProcMode = fread(fid, 1, '*uchar');
    fh.ucProcDirection = fread(fid, 1, '*uchar');
    fh.ucIsSmoothed = fread(fid, 1, '*uchar');
    fh.ucDataInterval = fread(fid, 1, 'double');
    fh.szDatum = fread(fid, [1,64], '*uchar');
    fh.dEpoch = fread(fid, 1, 'double');
    fh.dProgVersion = fread(fid, 1, 'double');
    fh.ullOffsetToFtr = fread(fid, 1, '*uint64');
    fh.ucDov = fread(fid, 1, '*uchar');
    fh.szReserved = fread(fid, [1,396], '*char');
    numrec = double((fh.ullOffsetToFtr-512)/224);
    %%
    gu1 = cgrecord(fid);
    t1 = gu1.dGpsTime;
    pos0 = [gu1.dLatitude; gu1.dLongitude; gu1.dEllHt];
    s = fseek(fid, double((numrec-2)*224), 'cof');
    gu2 = cgrecord(fid);
    t2 = gu2.dGpsTime;
    fseek(fid, 512, 'bof');
    
function gu = cgrecord(fid)
    gu.usSyncByte = fread(fid, 1, '*uint16');  % Epoch Header Record
    gu.dGpsTime = fread(fid, 1, 'double');
    gu.usWeekNum = fread(fid, 1, '*uint16');
    gu.usRecFlag = fread(fid, 1, '*uint16');
    gu.szReserved = fread(fid, [1,2], '*uchar');
    gu.szReserved0 = fread(fid, [1,11], '*uchar');  % GNSS Update Record
    gu.dCorrTime = fread(fid, 1, 'double');
    gu.dClockOffset = fread(fid, 1, 'double');
    gu.szReserved1 = fread(fid, [1,4], '*uchar');
    gu.dLatitude = fread(fid, 1, 'double');
    gu.dLongitude = fread(fid, 1, 'double');
    gu.dEllHt = fread(fid, 1, 'double');
    gu.AntHt = fread(fid, 1, '*single');
    gu.ucQuality = fread(fid, 1, '*uchar');
    gu.ucNumGPS = fread(fid, 1, '*uchar');
    gu.ucNumGLO = fread(fid, 1, '*uchar');
    gu.ucNumBDS = fread(fid, 1, '*uchar');
    gu.ucNumGAL = fread(fid, 1, '*uchar');
    gu.ucNumQZSS = fread(fid, 1, '*uchar');
    gu.szReserved2 = fread(fid, [1,8], '*uchar');
    gu.ucAmbStatus = fread(fid, 1, '*uchar');
    gu.usRMS_Code = fread(fid, 1, '*uint16');
    gu.usRMS_Phase = fread(fid, 1, '*uint16');
    gu.usRMS_Doppler = fread(fid, 1, '*uint16');
    gu.szReserved3 = fread(fid, [1,6], '*uchar');
    gu.usPDOP = fread(fid, 1, '*uint16');
    gu.usHDOP = fread(fid, 1, '*uint16');
    gu.usVDOP = fread(fid, 1, '*uint16');
    gu.szReserved4 = fread(fid, [1,14], '*uchar');
    gu.fLLVE = fread(fid, 1, '*single');
    gu.fLLVN = fread(fid, 1, '*single');
    gu.fLLVZ = fread(fid, 1, '*single');
    gu.fLLAE = fread(fid, 1, '*single');
    gu.fLLAN = fread(fid, 1, '*single');
    gu.fLLAZ = fread(fid, 1, '*single');
    gu.fSDE = fread(fid, 1, '*single');
    gu.fSDN = fread(fid, 1, '*single');
    gu.fSDZ = fread(fid, 1, '*single');
    gu.fCEN = fread(fid, 1, '*single');
    gu.fCNZ = fread(fid, 1, '*single');
    gu.fCEZ = fread(fid, 1, '*single');
    gu.szReserved5 = fread(fid, [1,5], '*uchar');
    gu.fSDVE = fread(fid, 1, '*single');
    gu.fSDVN = fread(fid, 1, '*single');
    gu.fSDVZ = fread(fid, 1, '*single');
    gu.fCVEN = fread(fid, 1, '*single');
    gu.fCVNZ = fread(fid, 1, '*single');
    gu.fCVEZ = fread(fid, 1, '*single');
    gu.szReserved6 = fread(fid, [1,25], '*uchar');

function cgcopy(fw, fr, len)
    if ~isstruct(fr)
        if nargin<3, len=inf; end
        data = fread(fr, [1,len], '*uchar');
        fwrite(fw, data, 'uchar');
        return;
    end
    fds = fieldnames(fr);
    for k=1:length(fds)
        data = getfield(fr, fds{k});
        fwrite(fw, data, class(data));
    end    