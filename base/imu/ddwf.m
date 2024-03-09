function [wfdot, wfdotdot] = ddwf(wf, afa)
global ddwf_wf_1  ddwf_dwf  ddwf_dwf_1  ddwf_ddwf ddwf_ts  ddwf_k
    if length(wf)==1  % initialize, ddwf(ts)
        ddwf_ts = wf; ddwf_k = 1;
        ddwf_dwf = zeros(6,1); ddwf_ddwf = zeros(6,1);
        return;
    end
    if nargin<2, afa=1; end
    if ddwf_k==1
        ddwf_wf_1 = wf;
        ddwf_dwf_1 = zeros(6,1);
    end
	ddwf_dwf = (1-afa)*ddwf_dwf + (wf-ddwf_wf_1)*(afa/ddwf_ts);
	ddwf_wf_1 = wf;
	ddwf_ddwf = (1-afa)*ddwf_ddwf + (ddwf_dwf-ddwf_dwf_1)*(afa/ddwf_ts);
	ddwf_dwf_1 = ddwf_dwf;
    ddwf_k = ddwf_k + 1;
    wfdot = ddwf_dwf;  wfdotdot = ddwf_ddwf;


