function pos2bd(poss, fname, dis)
% trans pos(lat,lon) to Baidu map file 'xxx.html'
global glv
    if nargin<3, dis=10; end
    if nargin<2, fname=['PSINS_BaiduMap_',datestr(now,'HHMMSS'),'.html']; end
    if ~ischar(fname), dis=fname; fname=['PSINS_BaiduMap_',datestr(now,'HHMMSS'),'.html']; end  % pos2bd(pos, dis)
    if ~iscell(poss), poss={poss}; end 
    
    %map
    fmap = fopen(fname, 'wt');
    ftemp = fopen([glv.datapath,'BD_map_template.txt'], 'rt');
    copytemplate(fmap, ftemp, 17);
    
    for k=1:length(poss)
        pos = extrapos(poss{k}, dis);
        AddPointArr(fmap, pos, k);
    end
    
    fprintf(fmap, '\n\tvar point = new BMap.Point(pointArr1[0][0], pointArr1[0][1]);\n');
    fprintf(fmap, '\tmap.addOverlay(new BMap.Marker(point));\n');         
    fprintf(fmap, '\tmap.centerAndZoom(point, 13);\n');
    fprintf(fmap, '</script>');    
    fclose(fmap);   fclose(ftemp);
    
function AddPointArr(fid, pos, k)
    sty = { '{strokeColor:"red", strokeWeight:4, strokeOpacity:0.4}', ...
            '{strokeColor:"green", strokeWeight:4, strokeOpacity:0.4}', ...
            '{strokeColor:"blue", strokeWeight:4, strokeOpacity:0.4}', ...
            '{strokeColor:"yellow", strokeWeight:4, strokeOpacity:0.4}', ...
           };
    gps_bd09 = pos2gpsbd09(pos);
    fprintf(fid, '\n\t\tvar pointArr%d = [\n', k);
    M = length(gps_bd09);
    for i=1:M-1
        fprintf(fid, '[%.6f,%.6f],\n', gps_bd09(i,[2,1]));
    end
    fprintf(fid, '[%.6f,%.6f]\n', gps_bd09(M,[2,1]));
    fprintf(fid, '\t\t];\n');
    fprintf(fid, '\tfor (var i = 0; i < pointArr%d.length-1; i++) {\n', k);
	fprintf(fid, '\t\tvar trackPoint1= new BMap.Point(pointArr%d[i][0],pointArr%d[i][1]);\n', k,k);
	fprintf(fid, '\t\tvar trackPoint2= new BMap.Point(pointArr%d[i+1][0],pointArr%d[i+1][1]);\n',k,k);
	fprintf(fid, '\t\tvar polyline = new BMap.Polyline([trackPoint1,trackPoint2], %s);\n', sty{k});
	fprintf(fid, '\t\tmap.addOverlay(polyline);\n');
	fprintf(fid, '\t}\n');

function gps_bd09 = pos2gpsbd09(pos)    
    a = 6378245.0 ; 
    ee = 0.00669342162296594323  ;
    wag_lat=rad2deg(pos(:,end-3));wag_lon=rad2deg(pos(:,end-2));

    x=wag_lon-105.0;y=wag_lat-35.0;
    dLat = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y.^2+ 0.1 * x.* y + 0.2 * sqrt(abs(x));
    dLat =dLat + (20.0 * sin(6.0 * x * pi) + 20.0 *sin(2.0 * x * pi)) * 2.0 / 3.0;
    dLat =dLat + (20.0 * sin(y * pi) + 40.0 * sin(y / 3.0 * pi)) * 2.0 / 3.0;
    dLat= dLat + (160.0 * sin(y / 12.0 * pi) + 320 * sin(y * pi / 30.0)) * 2.0 / 3.0;

    dLon = 300.0 + x + 2.0 * y + 0.1 * x.^2 + 0.1 * x.* y + 0.1 * sqrt(abs(x));
    dLon = dLon + (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
    dLon = dLon + (20.0 * sin(x * pi) + 40.0 * sin(x / 3.0 * pi)) * 2.0 / 3.0;
    dLon = dLon + (150.0 * sin(x / 12.0 * pi) + 300.0 * sin(x / 30.0 * pi)) * 2.0 / 3.0;

    radLat=wag_lat./180.0*pi;
    magic=sin(radLat);
    magic=1-ee*magic.*magic;
    sqrtMagic=sqrt(magic);
    dLat=(dLat*180.0)./((a*(1-ee))./(magic.*sqrtMagic)*pi);
    dLon=(dLon*180.0)./((a./sqrtMagic).*cos(radLat)*pi);
    gc_lat=wag_lat+dLat;
    gc_lon=wag_lon+dLon;
    gps_gcj02=[gc_lat, gc_lon];

    %gcj02_to_bd09
    x_pi=3.14159265358979324 * 3000.0 / 180.0;
    x1=gps_gcj02(:,2);y1=gps_gcj02(:,1);
    z=sqrt(x1.^2+y1.^2)+0.00002*sin(y1.*x_pi);
    theta=atan2(y1,x1)+0.000003*cos(x1.*x_pi);
    bd_lon=z.*cos(theta)+0.0065;
    bd_lat=z.*sin(theta)+0.006;
    gps_bd09=[bd_lat,bd_lon];

function copytemplate(fout, fin, lines)
    if nargin<3, lines=1000; end
    for k=1:lines
        tline=fgetl(fin);
        if ~ischar(tline), break, end 
        fprintf(fout,[tline,'\n']);
    end
