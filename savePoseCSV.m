function savePoseCSV(src, filename)
    fid = fopen(filename, 'w');
    for i = 1:size(src,2)  %every col
        fprintf(fid, '%0.12f, %0.12f, %0.12f, %0.12f, %0.12f, %0.12f\n', src(:, i));
end

