function T = test_readStepData(T)
%SIMLAB_TESTS.TEST_READSTEPDATA  Does the CSV reader handle the traps it names?
%
% readStepData exists because historian exports come with headers, semicolons,
% European decimal commas, epoch timestamps and shuffled columns, and each of
% those silently produces a model that is wrong by a factor. So every one of
% those traps gets its own fixture here, written to a temp file, read back,
% and checked against the truth that was written.

    tmp = tempdir;

    % ==================================================================
    % 1. plain CSV, numeric, no header
    % ==================================================================
    f1 = fullfile(tmp, 'simlab_plain.csv');
    fid = fopen(f1, 'w');
    for k = 0:49
        fprintf(fid, '%.6g,%.6g,%.6g\n', k * 0.1, 2 * (1 - exp(-k * 0.1 / 3)), ...
            (k >= 10) * 1.0);
    end
    fclose(fid);

    d1 = simlab.readStepData(f1, struct());
    T = simlab_tests.eq(T, d1.rows, 50, 'all 50 rows survive');
    T = simlab_tests.near(T, d1.dt, 0.1, 1e-9, 'dt is read from the data');
    T = simlab_tests.near(T, d1.t(11), 1.0, 1e-9, 'the time column is column 1');
    T = simlab_tests.near(T, d1.y(1), 0, 1e-9, 'measurement is column 2');
    T = simlab_tests.ok(T, ~isempty(d1.u), ...
        'a third numeric column is picked up as the command automatically');
    T = simlab_tests.near(T, d1.u(11), 1.0, 1e-9, 'and it is the right column');

    % ==================================================================
    % 2. header row + column names instead of indices
    % ==================================================================
    f2 = fullfile(tmp, 'simlab_header.csv');
    fid = fopen(f2, 'w');
    fprintf(fid, 'Time [s],Temp [degC],Cmd\n');
    for k = 0:39
        fprintf(fid, '%.6g,%.6g,%.6g\n', k * 0.5, 100 + 5 * k * 0, 20);
    end
    fclose(fid);

    d2 = simlab.readStepData(f2, struct('tCol', 'Time', 'yCol', 'Temp'));
    T = simlab_tests.eq(T, d2.rows, 40, 'the header row is dropped, not parsed as data');
    T = simlab_tests.near(T, d2.t(3), 1.0, 1e-9, 'columns resolve by name');
    T = simlab_tests.near(T, d2.y(3), 100, 1e-9, 'including the measurement');

    % ==================================================================
    % 3. semicolon delimiter with European decimal commas
    % ==================================================================
    % t = k, y = 2*k + 0.5, written with a decimal comma and a semicolon
    % delimiter: the two traps at once.
    f3 = fullfile(tmp, 'simlab_euro.csv');
    fid = fopen(f3, 'w');
    for k = 0:29
        fprintf(fid, '%d;%d,5\n', k, 2 * k);
    end
    fclose(fid);

    d3 = simlab.readStepData(f3, struct('delimiter', ';'));
    T = simlab_tests.near(T, d3.y(3), 2 * 2 + 0.5, 1e-9, ...
        'a decimal comma becomes a decimal point: "4,5" reads as 4.5');
    T = simlab_tests.near(T, d3.t(4), 3, 1e-9, 'the semicolon is the field separator');

    % ==================================================================
    % 4. time units and epoch-like timestamps
    % ==================================================================
    f4 = fullfile(tmp, 'simlab_ms.csv');
    fid = fopen(f4, 'w');
    for k = 0:49
        fprintf(fid, '%.6g,%.6g\n', k * 100, 1 - exp(-k * 100 / 3000));
    end
    fclose(fid);

    d4 = simlab.readStepData(f4, struct('timeUnit', 'ms'));
    T = simlab_tests.near(T, d4.dt, 0.1, 1e-9, ...
        'timeUnit=ms converts a 100 ms grid to 0.1 s');
    T = simlab_tests.near(T, d4.t(1), 0, 1e-12, 'and the origin is the first sample');

    % An epoch column is converted to relative seconds, with a warning rather
    % than a silent reinterpretation.
    f5 = fullfile(tmp, 'simlab_epoch.csv');
    fid = fopen(f5, 'w');
    for k = 0:29
        fprintf(fid, '%d,%.6g\n', 1700000000 + k * 10, 0.5 * k);
    end
    fclose(fid);
    w = warning('query', 'simlab:readStepData:epoch');
    warning('off', 'simlab:readStepData:epoch');
    d5 = simlab.readStepData(f5);
    warning('on', 'simlab:readStepData:epoch');
    T = simlab_tests.near(T, d5.t(2), 10, 1e-9, ...
        'an epoch column becomes relative seconds (10 s apart)');
    T = simlab_tests.near(T, d5.t(1), 0, 1e-12, 'starting at zero');

    % ==================================================================
    % 5. decimation
    % ==================================================================
    f6 = fullfile(tmp, 'simlab_dec.csv');
    fid = fopen(f6, 'w');
    for k = 0:99
        fprintf(fid, '%.6g,%.6g\n', k * 0.01, k);
    end
    fclose(fid);
    d6 = simlab.readStepData(f6, struct('decimate', 5));
    T = simlab_tests.eq(T, d6.rows, 20, 'decimate 5 keeps every fifth row');
    T = simlab_tests.near(T, d6.t(2), 0.05, 1e-9, 'spaced five samples apart');

    % ==================================================================
    % 6. the refusals
    % ==================================================================
    % Non-monotone time must be an error here, not a strange model later.
    f7 = fullfile(tmp, 'simlab_bad.csv');
    fid = fopen(f7, 'w');
    fprintf(fid, '0,1\n2,2\n1,3\n3,4\n');
    for k = 4:29
        fprintf(fid, '%d,%d\n', k, k);
    end
    fclose(fid);
    threw = false;
    try
        simlab.readStepData(f7);
    catch err
        threw = ~isempty(strfind(err.identifier, 'order')); %#ok<STREMP>
    end
    T = simlab_tests.ok(T, threw, ...
        'a time column that runs backwards is refused with :order');

    % A record that is too short to fit moments.
    f8 = fullfile(tmp, 'simlab_short.csv');
    fid = fopen(f8, 'w');
    for k = 0:4
        fprintf(fid, '%.6g,%.6g\n', k, k);
    end
    fclose(fid);
    threw2 = false;
    try
        simlab.readStepData(f8);
    catch err
        threw2 = ~isempty(strfind(err.identifier, 'short')); %#ok<STREMP>
    end
    T = simlab_tests.ok(T, threw2, 'five rows is refused with :short');

    % A missing file is named, not a generic open error.
    threw3 = false;
    try
        simlab.readStepData(fullfile(tmp, 'simlab_nofile_xyz.csv'));
    catch err
        threw3 = ~isempty(strfind(err.identifier, 'noFile')); %#ok<STREMP>
    end
    T = simlab_tests.ok(T, threw3, 'a missing file is refused with :noFile');

    % ==================================================================
    % 7. the hand-off to identify
    % ==================================================================
    % The struct readStepData returns must drop straight into identify.
    fid = fopen(fullfile(tmp, 'simlab_ident.csv'), 'w');
    for k = 0:5999
        t = k * 0.1;
        u = 20 + 30 * (t >= 50);
        y = 0;
        if t > 62
            y = 2 * 30 * (1 - exp(-(t - 62) / 45));
        end
        fprintf(fid, '%.6g,%.6g,%.6g\n', t, y, u);
    end
    fclose(fid);
    dd = simlab.readStepData(fullfile(tmp, 'simlab_ident.csv'));
    m = simlab.identify(dd);
    T = simlab_tests.near(T, m.t, 45, 0.05, ...
        'the reader-then-fitter chain recovers tau = 45 s from a file');
    T = simlab_tests.near(T, m.l, 12, 0.15, 'and L = 12 s');
    T = simlab_tests.near(T, m.k, 2, 0.10, 'and K = 2');

    % clean up
    delete(fullfile(tmp, 'simlab_*.csv'));
end
