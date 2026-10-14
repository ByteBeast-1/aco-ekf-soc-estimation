function ok = run_everything(mode)
%RUN_EVERYTHING Single unified pipeline entry point for BMS project.
%   ok = run_everything() runs full pipeline:
%     1. build_bms_models: Programmatically generates models & dictionary
%     2. run_bms_regression: Runs complete regression test suite
%     3. run_all: Runs scenario sweeps and exports time-series CSVs & plots
%     4. bms_cross_matrix: Executes cross-feeding test matrix and plots
%     5. make_results_table: Regenerates README markdown results tables from CSV
%
%   ok = run_everything('quick') runs quick verification pipeline.
%   Exits with an error if any stage fails (CI-ready).

    here = fileparts(mfilename('fullpath'));
    root_dir = fullfile(here, '..');
    addpath(here);
    addpath(fullfile(here, 'lib'));

    if nargin < 1 || isempty(mode)
        mode = 'full';
    end

    t_start = tic;
    fprintf('=========================================================================\n');
    fprintf(' BMS UNIFIED PIPELINE RUNNER (run_everything)\n');
    fprintf(' Mode: %s\n', upper(mode));
    fprintf('=========================================================================\n\n');

    % 1. Build models
    fprintf('--> [1/5] Building all Simulink models and data dictionary...\n');
    try
        build_bms_models;
        fprintf('    [OK] Model build completed successfully.\n\n');
    catch ME
        error('run_everything:BuildFailed', 'Model build failed:\n%s', ME.message);
    end

    % 2. Run regression
    fprintf('--> [2/5] Running automated regression test suite...\n');
    try
        ok_reg = run_bms_regression;
        if ~ok_reg
            error('run_everything:RegressionFailed', 'Regression checks failed.');
        end
        fprintf('    [OK] All regression checks passed.\n\n');
    catch ME
        error('run_everything:RegressionFailed', 'Regression execution failed:\n%s', ME.message);
    end

    % 3. Run scenario batch sweeps
    fprintf('--> [3/5] Running scenario sweeps (run_all)...\n');
    try
        if strcmpi(mode, 'quick')
            run_all('quick');
        else
            run_all('all');
        end
        fprintf('    [OK] Scenario sweeps completed successfully.\n\n');
    catch ME
        error('run_everything:RunAllFailed', 'Scenario sweep execution failed:\n%s', ME.message);
    end

    % 4. Run cross-feeding test matrix
    fprintf('--> [4/5] Executing cross-feeding test matrix...\n');
    try
        if strcmpi(mode, 'quick')
            bms_cross_matrix('quick');
        else
            bms_cross_matrix('full');
        end
        fprintf('    [OK] Cross-feeding matrix generated successfully.\n\n');
    catch ME
        error('run_everything:CrossMatrixFailed', 'Cross-matrix generation failed:\n%s', ME.message);
    end

    % 5. Regenerate documentation results table
    fprintf('--> [5/5] Updating README results tables from verified CSV...\n');
    py_tool = fullfile(root_dir, 'tools', 'make_results_table.py');
    readme_path = fullfile(root_dir, 'README.md');
    csv_path = fullfile(here, 'results', 'bms_regression_results.csv');
    cross_csv = fullfile(here, 'results', 'cross_matrix.csv');

    cmd = sprintf('python "%s" --csv "%s" --cross-csv "%s" --update "%s"', ...
        py_tool, csv_path, cross_csv, readme_path);
    [status, cmdout] = system(cmd);
    if status ~= 0
        cmd_py = sprintf('py "%s" --csv "%s" --cross-csv "%s" --update "%s"', ...
            py_tool, csv_path, cross_csv, readme_path);
        [status, cmdout] = system(cmd_py);
    end

    if status ~= 0
        error('run_everything:DocTableUpdateFailed', 'Failed to update README results tables:\n%s', cmdout);
    else
        fprintf('    %s\n', strtrim(cmdout));
        fprintf('    [OK] Documentation updated from verified CSV.\n\n');
    end

    elapsed = toc(t_start);
    fprintf('=========================================================================\n');
    fprintf(' BMS UNIFIED PIPELINE COMPLETED SUCCESSFULLY in %.1f seconds.\n', elapsed);
    fprintf('=========================================================================\n');

    ok = true;
end
