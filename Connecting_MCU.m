function [imu] = run_imu_ukf_ready()
% RUN_IMU_UKF_READY
% Streams STM32 9-DoF IMU CSV and stores values as DOUBLE.
% Structured for direct UKF usage.

BAUD = 115200;
PRINT_HZ = 10;

% Close any existing serial connections
try
    delete(serialportfind);
end

ports = string(serialportlist("available"));
if isempty(ports)
    error("No available COM ports found.");
end

disp("Available ports:");
disp(ports);

s = [];
for p = ports
    try
        tmp = serialport(p, BAUD, "Timeout", 0.2);
        configureTerminator(tmp, "LF");
        flush(tmp);
        pause(0.6);

        t0 = tic;
        ok = false;

        while toc(t0) < 2.0
            if tmp.NumBytesAvailable > 0
                line = strtrim(readline(tmp));
                if startsWith(line,"t_us") || ...
                   (numel(split(line,",")) == 16)
                    ok = true;
                    break;
                end
            else
                pause(0.02);
            end
        end

        if ok
            s = tmp;
            fprintf("✅ Connected on %s\n", p);
            break;
        else
            clear tmp
        end
    catch
        try, clear tmp; end
    end
end

if isempty(s)
    error("IMU stream not found.");
end

disp("Streaming... Ctrl+C to stop.");

% ---------- Preallocate ----------
MAX_SAMPLES = 200000; % adjust if needed
imu.time      = zeros(MAX_SAMPLES,1,'double');
imu.dt        = zeros(MAX_SAMPLES,1,'double');
imu.acc       = zeros(MAX_SAMPLES,3,'double');
imu.gyro      = zeros(MAX_SAMPLES,3,'double');
imu.mag       = zeros(MAX_SAMPLES,3,'double');
imu.accNorm   = zeros(MAX_SAMPLES,1,'double');
imu.gyroNorm  = zeros(MAX_SAMPLES,1,'double');
imu.magNorm   = zeros(MAX_SAMPLES,1,'double');
imu.temp      = zeros(MAX_SAMPLES,1,'double');
imu.zupt      = zeros(MAX_SAMPLES,1,'double');

k = 0;
t0_us = [];
lastPrint = tic;

% ---------- Streaming Loop ----------
while true
    if s.NumBytesAvailable > 0
        line = strtrim(readline(s));

        if line == "" || startsWith(line,"#")
            continue;
        end

        parts = split(line,",");
        if numel(parts) ~= 16
            continue;
        end

        v = str2double(parts);  % <-- DOUBLE conversion happens here
        if any(isnan(v))
            continue;
        end

        k = k + 1;
        if k > MAX_SAMPLES
            warning("Buffer full.");
            break;
        end

        if isempty(t0_us)
            t0_us = v(1);
        end

        imu.time(k) = (v(1) - t0_us) * 1e-6;
        imu.dt(k)   = v(2) * 1e-6;

        imu.acc(k,:)  = v(3:5);
        imu.gyro(k,:) = v(6:8);
        imu.mag(k,:)  = v(9:11);

        imu.accNorm(k)  = v(12);
        imu.gyroNorm(k) = v(13);
        imu.magNorm(k)  = v(14);

        imu.temp(k) = v(15);
        imu.zupt(k) = v(16);

        % Rate-limited print
        if toc(lastPrint) > (1/PRINT_HZ)
            lastPrint = tic;
            fprintf("t=%.3f | acc=[%.2f %.2f %.2f] | gyro=[%.3f %.3f %.3f] | mag=[%.1f %.1f %.1f]\n", ...
                imu.time(k), ...
                imu.acc(k,1), imu.acc(k,2), imu.acc(k,3), ...
                imu.gyro(k,1), imu.gyro(k,2), imu.gyro(k,3), ...
                imu.mag(k,1), imu.mag(k,2), imu.mag(k,3));
        end
    else
        pause(0.01);
    end
end

% Trim unused preallocation
fields = fieldnames(imu);
for i = 1:length(fields)
    imu.(fields{i}) = imu.(fields{i})(1:k,:);
end

assignin('base','imu',imu);

disp("✅ IMU data saved to workspace as struct: imu");

end