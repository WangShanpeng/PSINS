function [obs, obss] = findgpsobs(obss)
% see also  getgnssvp.
global glb_obss glb_obss_idx
    if length(obss)>1, glb_obss=obss; return;
    elseif obss<0, obss=min(glb_obss(:,1));
    elseif obss==inf, obss=max(glb_obss(:,1)); end
    glb_obss_idx = find(glb_obss(:,1)==obss);
    if isempty(glb_obss_idx), obs = []; 
    else,  obs = glb_obss(glb_obss_idx,:); end
    
    
    
    