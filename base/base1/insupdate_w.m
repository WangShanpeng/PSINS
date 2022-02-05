function ins = insupdate_w(ins, imu)
% Wander-azimuth SINS Updating Alogrithm.
% Ref. 'Qin Yongyuan, Inertial Navigation, Ch.8.1.3' 
%
% Prototype: ins = insupdate_f(ins, imu)
% Inputs: ins - SINS structure array created by function 'insinit'
%         imu - gyro & acc incremental sample(s)
% Output: ins - SINS structure array after updating,
%         where, ins.afa_w=wander azimuth, is counter-clock-wise positive
%                ins.vT platform velocity
%                ins.CTe platform transformation matrix
%
% See also  insinit, insupdate, insupdate_f.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 30/10/2021
    if isfield(ins,'afa_w0')  % set ins.afa_w0, then init wander-azimuth SINS
        ins.afa_w = ins.afa_w0; ins = rmfield(ins,'afa_w0');
        ins.afa_wDot = 0;
        CTn = rv2m([0;0;ins.afa_w]);
        ins.vT = CTn*ins.vn;
        ins.CTe = CTn*pos2cen(ins.pos)';
    end
    nn = size(imu,1);
    nts = nn*ins.ts;  nts2 = nts/2;  ins.nts = nts;
    [phim, dvbm] = cnscl(imu,0);    % coning & sculling compensation
    phim = ins.Kg*phim-ins.eb*nts; dvbm = ins.Ka*dvbm-ins.db*nts;  % calibration
    %% earth & angular rate updating 
    vn01 = ins.vn+ins.an*nts2; pos01 = ins.pos+ins.Mpv*vn01*nts2;  % extrapolation at t1/2
    if ins.openloop==0, ins.eth = ethupdate(ins.eth, pos01, vn01);
    elseif ins.openloop==1, ins.eth = ethupdate(ins.eth, ins.pos0, ins.vn0); end
    ins.wib = phim/nts; ins.fb = dvbm/nts;  % same as trjsimu
    ins.web = ins.wib - ins.Cnb'*ins.eth.wnie;
    ins.wnb = ins.wib - (ins.Cnb*rv2m(phim/2))'*ins.eth.wnin;  % 2014-11-30
    %% (1)velocity updating
    CTn = a2mat([0;0;-ins.afa_w-ins.afa_wDot*nts/2]); % v.s. Cnb=a2mat([0;0;ins.afa_w])
    ins.fn = qmulv(ins.qnb, ins.fb);
    ins.an = rotv(-ins.eth.wnin*nts2, ins.fn) + ins.eth.gcc;
    ins.vn = ins.vn + ins.an*nts;
	ins.vT = CTn*ins.vn;
    %% (2)position updating
    ins.Mpv(4)=1/ins.eth.RMh; ins.Mpv(2)=1/ins.eth.clRNh;
    ins.afa_wDot = -ins.vn(1)/ins.eth.RNh*ins.eth.tl;   % (8.1.37) afa_fDot
    wTeT = CTn*ins.eth.wnen + [0;0;ins.afa_wDot];
    ins.CTe = mupdt(ins.CTe, -ins.CTe'*wTeT*nts);
    ins.pos = [asin(ins.CTe(3,3)); atan2(ins.CTe(3,2),ins.CTe(3,1)); ins.pos(3)+ins.vT(3)*nts];   % (8.1.39a/b)
    ins.afa_w = atan2(ins.CTe(1,3),ins.CTe(2,3));   % (8.1.39c)
    %% (3)attitude updating
    ins.qnb = qupdt2(ins.qnb, phim, ins.eth.wnin*nts);
    [ins.qnb, ins.att, ins.Cnb] = attsyn(ins.qnb);
    ins.avp = [ins.att; ins.vn; ins.pos];

