function [Kg, eb, Ka, db, data] = ldpara(fname, dis)
global glv
    ext = fname(end-2:end);
    switch ext
        case 'bin'
            fid = fopen(fname, 'rb');
            data = double(fread(fid, [40,1], 'float'));
            fclose(fid);
        case 'txt'
            data = load(fname);
            data = [data(1,:), data(2,:), data(3,:)]';
    end
    Kg = reshape(data( 1: 9),3,3)';  eb = data(10:12)';
    Ka = reshape(data(13:21),3,3)';  db = data(22:24)';
    if nargin<2, dis=1; end
    if dis % display
        Kg, eb/glv.dps, Ka, db/glv.mg,
    end