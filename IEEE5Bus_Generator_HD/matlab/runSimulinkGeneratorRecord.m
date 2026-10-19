function result = runSimulinkGeneratorRecord(opts)
%RUNSIMULINKGENERATORRECORD  microgrid_case5.slx'i ayarlar, kosar, CSV yazar.
%
%   MATLAB + Simscape Electrical gerekir (Octave / bu Cloud ortaminda CALISMAZ).
%
%   runSimulinkGeneratorRecord
%   runSimulinkGeneratorRecord(struct('StopTime', 10, 'pmStepFrac', 0.08, 'gitPush', true))
%
%   Yaptiklari:
%     1. mg_params.mat yukler, .slx acar
%     2. Cozucu: ode14x, 50 us (ode4 kullanilmaz)
%     3. SM_Bus1/3/5 icine To Workspace ekler (yoksa): Pm_MW_b*, Pe_MW_b*, w_pu_b*
%        Kaynaklar: Pmech_MW, Pe_LPF, Int_omega  (kablo KOPARILMAZ)
%     4. SM_Bus3 Pm'sine Rate Limiter'li basamak (varsayilan +%8, t=1 s, 0.1 s rampa)
%     5. sim() kosar
%     6. outputs/data/Simulink_SM_Bus{1,3,5}.csv yazar, LS tahminini basar
%     7. Istegie bagli git add/commit/push (opts.gitCommit / opts.gitPush)
%
%   Onemli alanlar: StopTime (10), pmStepFrac (0.08), rampSec (0.1), D_pu ([]=model),
%   SampleTime (0.02), gitPush (false), gitCommit (true), pinn (false).
if nargin < 1, opts = struct(); end
opts = dfl(opts, 'StopTime', 10);
opts = dfl(opts, 'dt', 50e-6);
opts = dfl(opts, 'solver', 'ode14x');
opts = dfl(opts, 'SampleTime', 0.02);
opts = dfl(opts, 'pmBus', 3);
opts = dfl(opts, 'pmStepFrac', 0.08);
opts = dfl(opts, 'pmStepTime', 1.0);
opts = dfl(opts, 'rampSec', 0.1);
opts = dfl(opts, 'D_pu', []);          % bos = modeldeki degeri birak (1.5)
opts = dfl(opts, 'pinn', false);
opts = dfl(opts, 'gitCommit', true);
opts = dfl(opts, 'gitPush', false);
opts = dfl(opts, 'openModel', false);

if exist('sim', 'file') ~= 2 && exist('sim', 'builtin') ~= 5
    error(['Bu betik MATLAB/Simulink ister. Octave veya Cloud Agent''da kosmaz. ', ...
        'Yerel MATLAB''de: cd IEEE5Bus_Generator_HD/matlab; runSimulinkGeneratorRecord']);
end

here = fileparts(mfilename('fullpath'));
setupGeneratorPaths();
root = fileparts(here);                       % IEEE5Bus_Generator_HD
repo = fileparts(root);
inDir = fullfile(root, 'inputs');
slx = fullfile(inDir, 'SIMULINK', 'microgrid_case5.slx');
if exist(slx, 'file') ~= 2
    error('Model bulunamadi: %s', slx);
end

fprintf('=== Simulink jenerator kaydi ===\n');
fprintf('  model     : %s\n', slx);
fprintf('  StopTime  : %g s   solver %s @ %g s\n', opts.StopTime, opts.solver, opts.dt);
fprintf('  Pm step   : Bus %d  %+g%%  t=%g s  rampa=%g s\n', ...
    opts.pmBus, 100*opts.pmStepFrac, opts.pmStepTime, opts.rampSec);

% --- workspace parametreleri ---
mgFile = fullfile(inDir, 'mg_params.mat');
S = load(mgFile);
assignin('base', 'mg', S.mg);
if isfield(S, 'f0'), f0 = S.f0; else, f0 = S.mg.base.fbase; end
assignin('base', 'f0', f0);

[mdlPath, mdlName, ~] = fileparts(slx);
load_system(slx);
if opts.openModel, open_system(mdlName); end

% --- solver (IO_CONTRACTS: ode4 bu agda iraksar) ---
set_param(mdlName, 'SolverType', 'Fixed-step', ...
    'Solver', opts.solver, ...
    'FixedStep', num2str(opts.dt, 16), ...
    'StopTime', num2str(opts.StopTime), ...
    'ReturnWorkspaceOutputs', 'on', ...
    'ReturnWorkspaceOutputsName', 'out', ...
    'SaveOutput', 'on', ...
    'SignalLogging', 'on');

buses = [1 3 5];
sm = cell(1, 3);
Sbase = zeros(1, 3);
for k = 1:3
    sm{k} = sprintf('%s/TransmissionNetwork/SM_Bus%d', mdlName, buses(k));
    if getSimulinkBlockHandle(sm{k}) <= 0
        error('Alt sistem yok: %s  (.slx TransmissionNetwork/SM_Bus* icermiyor olabilir)', sm{k});
    end
    Sbase(k) = str2double(get_param([sm{k} '/Sbase_MW'], 'Value'));
    if ~isempty(opts.D_pu)
        set_param([sm{k} '/D_pu'], 'Value', num2str(opts.D_pu));
    end
    Dnow = str2double(get_param([sm{k} '/D_pu'], 'Value'));
    fprintf('  SM_Bus%d  Sbase=%.4f MW  D_pu=%g  H=%s\n', buses(k), Sbase(k), Dnow, ...
        get_param([sm{k} '/H_sec'], 'Value'));
    ensureLog(sm{k}, 'Pmech_MW', sprintf('log_Pm_b%d', buses(k)), sprintf('Pm_MW_b%d', buses(k)), opts.SampleTime);
    ensureLog(sm{k}, 'Pe_LPF',   sprintf('log_Pe_b%d', buses(k)), sprintf('Pe_MW_b%d', buses(k)), opts.SampleTime);
    ensureLog(sm{k}, 'Int_omega',sprintf('log_w_b%d', buses(k)),  sprintf('w_pu_b%d', buses(k)),  opts.SampleTime);
end

kPm = find(buses == opts.pmBus, 1);
if isempty(kPm), error('pmBus 1, 3 veya 5 olmali'); end
installPmRamp(sm{kPm}, opts.pmStepFrac, opts.pmStepTime, opts.rampSec);
% Kayit Pmech_MW constant'ta kalmasin; swing'e giden Pm_sum kaydedilsin.
logPm = sprintf('log_Pm_b%d', opts.pmBus);
disconnectIn(sm{kPm}, logPm, 1);
connectLine(sm{kPm}, 'Pm_sum/1', [logPm '/1']);

fprintf('Simulasyon basliyor...\n');
tic;
try
    simOut = sim(mdlName, 'StopTime', num2str(opts.StopTime));
catch err
    fprintf(2, 'Simulasyon durdu: %s\n', err.message);
    fprintf(2, ['Cozum: StopTime=2 ile deneyin; pmStepFrac=0.05; rampSec=0.2; ', ...
        'D_pu=30 (o zaman LS D~30 bulur); solver ode14x / ode1be, dt=50e-6.\n']);
    rethrow(err);
end
fprintf('Simulasyon bitti (%.1f s duvar saati).\n', toc);

dataDir = fullfile(root, 'outputs', 'data');
if exist(dataDir, 'dir') ~= 7, mkdir(dataDir); end
sumPath = fullfile(root, 'outputs', 'simulink_HD_estimates.csv');
fid = fopen(sumPath, 'w');
fprintf(fid, 'bus,Sbase_MW,wSpan,identifiable,LS_H,LS_D,c,csv\n');

result.simOut = simOut;
result.Sbase = Sbase;
result.csv = cell(1, 3);
for k = 1:3
    b = buses(k);
    Pm = tsget(simOut, sprintf('Pm_MW_b%d', b));
    Pe = tsget(simOut, sprintf('Pe_MW_b%d', b));
    w  = tsget(simOut, sprintf('w_pu_b%d', b));
    t = Pm.Time(:);
    f_Hz = f0 * w.Data(:);
    csv = fullfile(dataDir, sprintf('Simulink_SM_Bus%d.csv', b));
    simulinkLogToCsv(t, Pm.Data(:), Pe.Data(:), f_Hz, Sbase(k), f0, csv);
    est = estimateGeneratorFromCsv(csv, struct('pinn', opts.pinn, 'dampingRef', 1));
    result.csv{k} = csv;
    result.est(k) = est; %#ok<AGROW>
    fprintf(fid, '%d,%.6f,%.6e,%d,%.6f,%.6f,%.6g,%s\n', ...
        b, Sbase(k), est.ls.wSpan, est.ls.identifiable, est.ls.H, est.ls.D, est.free.c, csv);
end
fclose(fid);
fprintf('Ozet: %s\n', sumPath);

% modeli kaydetmeyin: To Workspace / rampa kullanici .slx'ine kalici yazilmasin
try, close_system(mdlName, 0); catch, end

if opts.gitCommit
    gitPublish(repo, opts.gitPush);
end
end

function opts = dfl(opts, name, value)
if ~isfield(opts, name) || isempty(opts.(name))
    opts.(name) = value;
end
end

function ts = tsget(simOut, name)
if isa(simOut, 'Simulink.SimulationOutput')
    try
        ts = simOut.get(name);
    catch
        ts = simOut.(name);
    end
else
    ts = evalin('base', name);
end
if ~isa(ts, 'timeseries')
    error('Sinyal %s timeseries degil (Single simulation output / Save format=Timeseries).', name);
end
end

function ensureLog(smPath, srcName, logName, varName, Ts)
logPath = [smPath '/' logName];
if getSimulinkBlockHandle(logPath) <= 0
    add_block('simulink/Sinks/To Workspace', logPath, ...
        'Position', [40 400 140 430]);
end
set_param(logPath, ...
    'VariableName', varName, ...
    'SaveFormat', 'Timeseries', ...
    'SampleTime', num2str(Ts), ...
    'MaxDataPoints', 'inf', ...
    'Decimation', '1');
% mevcut kabloyu koparmadan T baglantisi
phSrc = get_param([smPath '/' srcName], 'PortHandles');
hOut = phSrc.Outport(1);
phLog = get_param(logPath, 'PortHandles');
hIn = phLog.Inport(1);
already = false;
try
    ln = get_param(hIn, 'Line');
    already = ln > 0;
catch
end
if ~already
    add_line(smPath, hOut, hIn, 'autorouting', 'on');
end
end

function installPmRamp(smPath, frac, t0, rampt)
% Pmech_MW constant kalir; Step + RateLimiter + Sum ile (1 -> 1+frac).
% To Workspace (Pmech_MW'den T kablo) koparilmaz. Branched kabloda
% delete_line('src','dst') basarisiz olur; Swing_Eq girisinin line handle'i silinir.
if frac == 0, return; end
ensureBlk(smPath, 'Pm_step', 'simulink/Sources/Step', [40 330 80 360]);
ensureBlk(smPath, 'Pm_ramp', 'simulink/Discontinuities/Rate Limiter', [100 330 150 360]);
ensureBlk(smPath, 'Pm_gain', 'simulink/Math Operations/Gain', [170 330 210 360]);
ensureBlk(smPath, 'Pm_sum', 'simulink/Math Operations/Sum', [140 40 160 70]);
set_param([smPath '/Pm_sum'], 'Inputs', '++');

% Swing_Eq 1. giris hâlâ Pmech_MW (veya baska) ise kopar, Sum'a bagla.
if ~srcIs(smPath, 'Swing_Eq', 1, 'Pm_sum')
    disconnectIn(smPath, 'Swing_Eq', 1);
    connectLine(smPath, 'Pm_sum/1', 'Swing_Eq/1');
end
connectLine(smPath, 'Pmech_MW/1', 'Pm_sum/1');
connectLine(smPath, 'Pm_step/1', 'Pm_ramp/1');
connectLine(smPath, 'Pm_ramp/1', 'Pm_gain/1');
connectLine(smPath, 'Pm_gain/1', 'Pm_sum/2');

rising = max(abs(frac) / max(rampt, 1e-6), 1e-9);
set_param([smPath '/Pm_step'], 'Time', num2str(t0), 'Before', '0', 'After', '1');
set_param([smPath '/Pm_ramp'], 'RisingSlewLimit', num2str(rising), ...
    'FallingSlewLimit', num2str(-rising));
pm0 = str2double(get_param([smPath '/Pmech_MW'], 'Value'));
set_param([smPath '/Pm_gain'], 'Gain', num2str(pm0 * frac, 16));
fprintf('  Pm rampa: %g MW -> %g MW (Bus alt sistemi %s)\n', pm0, pm0*(1+frac), smPath);
end

function ensureBlk(smPath, name, lib, pos)
p = [smPath '/' name];
if getSimulinkBlockHandle(p) <= 0
    add_block(lib, p, 'Position', pos);
end
end

function disconnectIn(smPath, blk, inIdx)
ph = get_param([smPath '/' blk], 'PortHandles');
ln = get_param(ph.Inport(inIdx), 'Line');
if ~isempty(ln) && ln > 0
    delete_line(ln);
end
end

function tf = srcIs(smPath, blk, inIdx, srcName)
tf = false;
ph = get_param([smPath '/' blk], 'PortHandles');
ln = get_param(ph.Inport(inIdx), 'Line');
if isempty(ln) || ln < 0, return; end
src = get_param(ln, 'SrcBlockHandle');
if isempty(src) || src < 0, return; end
tf = strcmp(get_param(src, 'Name'), srcName);
end

function connectLine(smPath, srcPort, dstPort)
% dstPort 'Blk/k' — giris doluysa dokunma.
slash = find(dstPort == '/', 1, 'last');
blk = dstPort(1:slash-1);
k = str2double(dstPort(slash+1:end));
ph = get_param([smPath '/' blk], 'PortHandles');
ln = get_param(ph.Inport(k), 'Line');
if ~isempty(ln) && ln > 0
    return;
end
add_line(smPath, srcPort, dstPort, 'autorouting', 'on');
end

function gitPublish(repo, doPush)
if exist(repo, 'dir') ~= 7, warning('repo kok bulunamadi'); return; end
cmd = @(c) system(sprintf('git -C "%s" %s', repo, c));
cmd('add IEEE5Bus_Generator_HD/outputs/data/Simulink_SM_Bus*.csv IEEE5Bus_Generator_HD/outputs/simulink_HD_estimates.csv');
[st, out] = cmd('diff --cached --stat');
fprintf('%s', out);
if st ~= 0, return; end
[st2, ~] = cmd('diff --cached --quiet');
if st2 == 0
    fprintf('Git: yeni CSV yok (zaten commitli).\n');
    return;
end
[~, uname] = cmd('config user.name');
if isempty(strtrim(uname))
    fprintf(2, ['Git commit atlandi: kimlik yok. Bir kez:\n', ...
        '  !git config --global user.name "Ad Soyad"\n', ...
        '  !git config --global user.email "github-epostaniz@ornek.com"\n', ...
        'Sonra: !git commit ve !git push  (GIT_EL_KITABI.md)\n']);
    return;
end
msg = sprintf('Add Simulink SM_Bus H/D recordings (%s)', datestr(now, 31));
[stc, outc] = cmd(sprintf('commit -m "%s"', strrep(msg, '"', '''')));
fprintf('%s', outc);
if stc ~= 0
    fprintf(2, 'Git commit basarisiz (dosyalar yine outputs/data altinda).\n');
    return;
end
fprintf('Git commit: %s\n', msg);
if doPush
    [stp, outp] = cmd('push');
    fprintf('%s', outp);
    if stp ~= 0
        warning('git push basarisiz. MATLAB''de kimlik bilgisi yoksa klasorden: git push');
    end
end
end
