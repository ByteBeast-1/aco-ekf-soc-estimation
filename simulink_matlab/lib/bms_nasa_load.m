function D = bms_nasa_load(cellname, dis_idx, kind)
%BMS_NASA_LOAD Load one REAL cycle of a NASA Ames cell (data_raw/B000x.mat), resampled to 1 s.
%   D = BMS_NASA_LOAD('B0005', 1)            discharge cycle #1
%   D = BMS_NASA_LOAD('B0005', 1, 'charge')  real CC-CV charge cycle #1 (current tapers, negative sign)
%   fields (all column vectors, 1 Hz):
%       t [s] | I [A, discharge positive] | V [V] | T [K] | soc_ref [0..1] | cap_Ah | dt_raw [s]
%   soc_ref is Coulomb-counted from the measured current and normalised so it spans 1 -> 0 over the
%   discharge to cut-off (i.e. relative to the capacity actually delivered in that cycle).
    if nargin < 3,  kind = 'discharge';  end
    root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    f = fullfile(root, 'data_raw', [cellname '.mat']);
    if ~isfile(f)
        error('bms_nasa_load:missing', 'Missing %s', f);
    end
    s   = load(f);
    cyc = s.(cellname).cycle;
    di  = find(strcmp({cyc.type}, kind));
    if dis_idx > numel(di)
        error('bms_nasa_load:range', '%s has only %d %s cycles', cellname, numel(di), kind);
    end
    d  = cyc(di(dis_idx)).data;
    n  = min([numel(d.Time), numel(d.Current_measured), numel(d.Voltage_measured), ...
              numel(d.Temperature_measured)]);       % raw logs can differ in length by a sample
    t  = d.Time(1:n);  t = t(:) - t(1);
    I  = -d.Current_measured(1:n);  I = I(:);        % NASA logs discharge as negative
    V  = d.Voltage_measured(1:n);   V = V(:);
    Tc = d.Temperature_measured(1:n);  Tc = Tc(:);

    tg = (0:floor(t(end)))';                % uniform 1 s grid
    D.t = tg;
    D.I = interp1(t, I,  tg, 'linear');
    D.V = interp1(t, V,  tg, 'linear');
    D.T = interp1(t, Tc, tg, 'linear') + 273.15;

    q = cumtrapz(tg, D.I);
    if strcmp(kind, 'charge')
        D.soc_ref = q / q(end);              % charge: 0 -> 1 (assumes the cycle starts from ~empty)
    else
        D.soc_ref = 1.0 - q / q(end);
    end
    if isfield(d, 'Capacity'),  D.cap_Ah = d.Capacity;  else,  D.cap_Ah = NaN;  end
    D.dt_raw  = median(diff(t));
    D.cell    = cellname;
    D.cycle   = dis_idx;
    D.kind    = kind;
end
