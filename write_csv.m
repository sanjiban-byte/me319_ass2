function write_csv(fname, hdr, M, labels)
%WRITE_CSV  Minimal CSV writer (base MATLAB): header line, optional first-column labels.
fid = fopen(fname, 'w');  fprintf(fid, '%s\n', hdr);
for q = 1:size(M, 1)
    row = strjoin(arrayfun(@(v) sprintf('%.6g', v), M(q,:), 'UniformOutput', false), ',');
    if nargin > 3, fprintf(fid, '%s,%s\n', labels{q}, row); else, fprintf(fid, '%s\n', row); end
end
fclose(fid);
end
