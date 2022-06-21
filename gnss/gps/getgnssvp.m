function vp = getgnssvp(ephs, obss, tp, isfig)
% see also  findgpsobs.
global ggps
    findgpsobs(obss);
    recPos = [0,0,0,0]';
    if nargin<3, tp=-1; end
    for k=1:100
        [obsi,tp] = findgpsobs(tp);
        if size(obsi,1)<=4, tp=tp+1; continue; end
        [satpv, clkerr, TGD, orbitp] = satPosVelBatch(obsi(1,1)-obsi(:,3)/ggps.c, ephs(obsi(:,2),:));
        [pvt, vp, res] = lspvt(recPos, satpv, [obsi(:,3)+clkerr(:,2)*ggps.c]);
        break;
    end
    if nargin>=4
        AzEl = satPos2AzEl(satpv, vp(4:6));
        satplot(obsi(:,2), AzEl);
    end

    
    
    
    