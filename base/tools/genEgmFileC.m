function genEgmFileC(fname, C, S)
    fid = fopen(fname, 'w');
    fprintf(fid, '\ndouble egmc[]={\n');
    kk = 1;
    for k=1:length(C)
        for j=1:k
            fprintf(fid, '%.16e,', C(k,j));
            if mod(kk,10)==0, fprintf(fid, '\n'); end
            kk=kk+1;
        end
    end
    fprintf(fid, '\n};\n');
    
    fprintf(fid, '\ndouble egms[]={\n');
    kk = 1;
    for k=1:length(S)
        for j=1:k
            fprintf(fid, '%.16e,', S(k,j));
            if mod(kk,10)==0, fprintf(fid, '\n'); end
            kk=kk+1;
        end
    end
    fprintf(fid, '\n};\n');

    fclose(fid);