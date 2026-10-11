function exe = bms_sil_build(target, force)
%BMS_SIL_BUILD Compile the embedded firmware into a console program (no extra software needed).
%   exe = BMS_SIL_BUILD('sil')   build c_firmware/sil/sil_main.c + bms_ekf.c  -> c_firmware/build/bms_sil.exe
%   exe = BMS_SIL_BUILD('test')  build c_firmware/main_test.c  + bms_ekf.c    -> c_firmware/build/bms_test.exe
%   Uses gcc when it is on the path, otherwise the LCC-win64 compiler that ships inside MATLAB.
%   Warnings are treated as a failure under gcc (-Wall -Wextra), so the firmware stays warning-free.
    if nargin < 1,  target = 'sil';  end
    if nargin < 2,  force = false;   end
    root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    fw   = fullfile(root, 'c_firmware');
    bld  = fullfile(fw, 'build');
    if ~isfolder(bld),  mkdir(bld);  end

    if strcmp(target, 'sil')
        srcs = {fullfile(fw, 'sil', 'sil_main.c'), fullfile(fw, 'bms_ekf.c')};  exe = fullfile(bld, 'bms_sil.exe');
    else
        srcs = {fullfile(fw, 'main_test.c'), fullfile(fw, 'bms_ekf.c')};        exe = fullfile(bld, 'bms_test.exe');
    end

    newest_src = max(cellfun(@(f) dir(f).datenum, [srcs, {fullfile(fw, 'bms_ekf.h')}]));
    if ~force && isfile(exe) && dir(exe).datenum > newest_src,  return;  end
    if isfile(exe),  delete(exe);  end

    if system('gcc --version > NUL 2>&1') == 0
        cmd = sprintf('gcc -std=c99 -O2 -Wall -Wextra -Werror -o "%s" "%s" "%s" -lm', exe, srcs{1}, srcs{2});
        [st, out] = system(cmd);
        compiler = 'gcc';
    else
        lcc = fullfile(matlabroot, 'sys', 'lcc64', 'lcc64');
        if ~isfile(fullfile(lcc, 'bin', 'lcc64.exe'))
            error('bms_sil_build:nocompiler', 'No C compiler found: install gcc or use a MATLAB with the bundled LCC.');
        end
        setenv('PATH', [fullfile(lcc, 'bin') ';' getenv('PATH')]);
        objs = cell(size(srcs));  out = '';  st = 0;
        for i = 1:numel(srcs)
            [~, nm] = fileparts(srcs{i});  objs{i} = fullfile(bld, [nm '.obj']);
            cmd = sprintf('lcc64 -nodeclspec -Zp8 -c -noregistrylookup -I"%s" -Fo"%s" "%s"', fullfile(lcc, 'include64'), objs{i}, srcs{i});
            [s1, o1] = system(cmd);  out = [out o1];  st = st + s1;  %#ok<AGROW>
            if contains(o1, 'error', 'IgnoreCase', true) && ~contains(o1, ' 0 errors'),  st = st + 1;  end
        end
        if st == 0
            cmd = sprintf('lcclnk64 -subsystem console -L"%s" -o "%s" "%s" "%s"', fullfile(lcc, 'lib64'), exe, objs{1}, objs{2});
            [s2, o2] = system(cmd);  out = [out o2];  st = st + s2;
        end
        compiler = 'LCC-win64';
    end
    if st ~= 0 || ~isfile(exe)
        error('bms_sil_build:failed', 'Firmware build with %s failed:\n%s', compiler, out);
    end
end
