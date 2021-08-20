function fig2jpg(hfig, fname)
% Write figure to JPEG graphics file.
%
% Prototype: fig2jpg(fig, fname)
% Inputs: hfig - figure, handle
%         fname - file name to write
%
% See also  matbinfile.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 27/08/2020
    if nargin<2, fname=hfig; hfig=gcf; end
    frm = getframe(hfig);
    if isempty(strfind(fname, '.jgp')), fname = [fname,'.jpg']; end
    imwrite(frm.cdata, fname);
