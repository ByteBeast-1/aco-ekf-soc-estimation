function [I, soc0] = bms_profile(name)
%BMS_PROFILE Drive-cycle current [A, discharge positive] and true initial SOC for named scenarios.
    switch name
        case 'udds'      % urban dynamic cycle
            t = (0:1799);
            I = abs(1.0 + 0.8 * sin(2*pi*t/200) + 1.2 * (sin(2*pi*t/35) > 0.3));
            I = min(max(I, 0), 3.5);   soc0 = 0.85;
        case 'cc'        % highway cruise
            I = ones(1, 3600);         soc0 = 0.95;
        case 'pulse'     % stop-start city
            t = (0:2399);
            I = 2.0 * (mod(t, 90) < 60);   soc0 = 0.90;
        case 'cccv'      % fast charge, taper above 80 % SOC
            N = 3600;  I = zeros(1, N);
            for k = 1:N
                s = 0.20 + k / (N * 1.2);
                if s < 0.80;  I(k) = -2.0;  else;  I(k) = -(2.0 * (1 - s) / 0.20);  end
            end
            soc0 = 0.20;
        otherwise
            error('bms_profile:unknown', 'Unknown profile "%s"', name);
    end
end
