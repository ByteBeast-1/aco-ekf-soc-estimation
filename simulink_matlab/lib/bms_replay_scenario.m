function sc = bms_replay_scenario(cellname, dis_idx, init_err, kind)
%BMS_REPLAY_SCENARIO Wrap a real NASA discharge cycle as a bms_simulate scenario.
    if nargin < 3,  init_err = -0.15;  end
    if nargin < 4,  kind = 'discharge';  end
    D = bms_nasa_load(cellname, dis_idx, kind);
    sc.replay   = true;
    sc.I        = D.I(:)';
    sc.V        = D.V(:)';
    sc.T        = D.T(:)';
    sc.soc_ref  = D.soc_ref(:)';
    sc.init_err = init_err;
    if strcmp(kind, 'charge')
        sc.name = sprintf('%s charge #%d', cellname, dis_idx);
    else
        sc.name = sprintf('%s #%d (%.2f Ah)', cellname, dis_idx, D.cap_Ah);
    end
    sc.t        = D.t(:)';
    sc.cap_Ah   = D.cap_Ah;
end
