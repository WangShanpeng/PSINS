function [kgps, dt] = imugpssyn1(k0, k1, ForB)
% SIMU & GPS time synchronization. A schematic diagram for time  
% relationship between SIMU & GPS looks like
%                 k0               k1
%  imu_t:    -----|------*---|-----|--------
%                         <---dt--->     (Forward)
%                 <--dt-->               (Backward)
%  gps_t:    ------------|------------------
%                       kgps
%     where k0,k1 for SIMU data log index and kgps for GPS data log index.
% 
% Prototype: [kgps, dt] = imugpssyn(k0, k1, ForB)
% Usages:
%   For initialization:  imugpssyn(imut, gpst)
%       where imut is SIMU time array, gpst is GPS time array
%   For synchrony checking: [kgps, dt] = imugpssyn(k0, k1, ForB)
%       It checks if there is any GPS sample between SIMU time interval
%       imut(k0) and imut(k1), if exists, return the GPS index 'kgps'
%       and time gap 'dt'. 
%       ForB='F' for forward checking,
%       ForB='B' for backward checking, 
%       ForB='f' for re-setting from the first one,
%       ForB='b' for re-setting from the last one. 
%
% See also  insupdate, kfupdate, POSProcessing, combinedata, combinet, igsplot.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 03/02/2014
global igaln1
    if nargin==2  % initialization: imugpsaln(imut, gpst)
        igaln1.imut = k0; igaln1.gpst = k1;
        igaln1.glen = length(igaln1.gpst);
        igaln1.kgps = 1;
        return;
    end
    k0 = k0-1;
    if k0==0, k0 = 1; end
    t0 = igaln1.imut(k0); t1 = igaln1.imut(k1);
    kgps = 0; dt = 0;
    if ForB=='F'  % Forward search
        while igaln1.gpst(igaln1.kgps)<t0 
            igaln1.kgps = igaln1.kgps + 1;
            if igaln1.kgps>igaln1.glen
                igaln1.kgps = igaln1.glen;
                break;
            end
        end
        tg = igaln1.gpst(igaln1.kgps);
        if t0<tg && tg<=t1
            kgps = igaln1.kgps; dt = t1 - tg;
        end
    elseif ForB=='B' % Backward search
        while igaln1.gpst(igaln1.kgps)>t1 
            igaln1.kgps = igaln1.kgps - 1;
            if igaln1.kgps==0
                igaln1.kgps = 1;
                break;
            end
        end
        tg = igaln1.gpst(igaln1.kgps);
        if t0<=tg && tg<t1
            kgps = igaln1.kgps; dt = tg - t0;
        end
    elseif ForB=='f'  % Forward re-intialization, set to the first one
        igaln1.kgps = 1;
    elseif ForB=='b'  % Backward re-intialization, set to the last one
        igaln1.kgps = igaln1.glen;
    end