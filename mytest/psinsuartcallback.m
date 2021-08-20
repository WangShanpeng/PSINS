function psinsuartcallback(obj, event)
% Copyright(c) 2009-2018, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/08/2018
global glb
    [recbuf, cnt] = fread(obj, glb.recbuflen/4, 'float32');
    if ~ishandle(glb.hfig)
        if glb.fid>0
            fclose(glb.fid); glb.fid = 0;
        end
        fclose(glb.uart); delete(glb.uart);
        return;
    else
       fwrite(glb.fid, recbuf, 'float32');
       for ii=1:35
            if recbuf(ii)>9.364e13 && recbuf(ii)<9.365e13, % 0xaa55aa56
                break;
            end
       end
       if ii==35, return; end
       ki = ii+(0:34);
       glb.newbuf = [recbuf(ki), recbuf(ki+35*5), recbuf(ki+35*10), recbuf(ki+35*15)]';
       if glb.plotfirst==1
           glb.plotfirst = 0;
           glb.sz1newbuf = size(glb.newbuf,1)+1;
           dt = diff(glb.newbuf(end-1:end,2));
           glb.plotbuf(:,2) = (-size(glb.plotbuf,1):-1)'*dt + glb.newbuf(1,2);
       end
       glb.plotbuf = [glb.plotbuf(glb.sz1newbuf:end,:); glb.newbuf];
       figure(glb.hfig), 
       tstart = glb.plotbuf(1,2);  tend = glb.plotbuf(end,2);
       subplot(311), plot(glb.plotbuf(:,2), glb.plotbuf(:,3:5)); grid on
       xlim([tstart,tend]); ylabel('Gyro / \circ/s');
       subplot(312), plot(glb.plotbuf(:,2), glb.plotbuf(:,6:8)); grid on
       xlim([tstart,tend]); ylabel('Accel / m/s^2');
       subplot(313), plot(glb.plotbuf(:,2), glb.plotbuf(:,13:14)); grid on
       xlim([tstart,tend]); ylabel('Attitude / \circ');
       xlabel('t / s');
    end
