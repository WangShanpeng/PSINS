function genEgmFile(fname, GM, Re, ff, wie, C, S)
    fid = fopen(fname, 'w');
    fwrite(fid, GM, 'double');
    fwrite(fid, Re, 'double');
    fwrite(fid, ff, 'double');
    fwrite(fid, wie, 'double');
    fwrite(fid, length(C), 'int32');
    for k=1:length(C)
        fwrite(fid, C(k,1:k), 'double');
    end
    for k=1:length(S)
        fwrite(fid, S(k,1:k), 'double');
    end
    fclose(fid);
