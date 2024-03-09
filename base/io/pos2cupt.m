function pos2cupt(fname, pos, lv, dev)
% name(no_spaces) time(tow,s) lat(dec.deg) lon(dec.deg) hgt(m) lever_x(m) lever_y(m) lever_z(m) std_dev_x(m) std_dev_y(m) std_dev_z(m)
global glv
    if nargin<4, dev = 1; end
    if length(dev)==1; dev=[1;1;1]*dev; end
    if nargin<3, lv = 0; end
    if length(lv)==1; lv=[1;1;1]*lv; end
    if size(pos,2)>4, pos=pos(:,end-3:end); end  % avp or gps
    fid = fopen(fname, 'w');
    n = length(pos);
    if n>1000, pos=pos(1:1000,:);
    else pos(n:1000,end) = 9999; end
    n = 1000;  % max 1000 points
    fprintf(fid, '%d  %.3f  %.8f %.8f %.3f  %.3f %.3f %.3f  %.3f %.3f %.3f\n', ...
        [(1:n)', pos(:,end), pos(:,1:2)/glv.deg, pos(:,3), repmat(lv',n,1), repmat(dev',n,1)]');
    fclose(fid);
