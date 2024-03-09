function m = setdiag0(m)
% See also  bending.
    m = m-diag(diag(m));