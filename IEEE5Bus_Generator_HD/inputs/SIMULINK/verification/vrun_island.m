function R = vrun_island(caseName, Pld_MW, StopTime_s, isSmoke)
%% vrun_island — reusable full-model ISLAND verification engine (Section B)
% Loads the on-disk microgrid_case5.slx, sets an island scenario
% (CB_PCC=0, physical D_pu=1.5, P_load=Pld_MW @ pf 0.95 lagging), runs it at
% 50 us / ode14x with Simscape logging + SDI OFF (disk-fill safety), records
% own-rate taps (electrical dec=20 = 1 ms; EMS-native dec=1 = chart 0.1 s),
% saves RESULTS/verification/<caseName>.mat, and appends one PASS/FAIL line to
% RESULTS/verification/SUMMARY.txt.
%
%   R = vrun_island(caseName, Pld_MW, StopTime_s, isSmoke)
%     caseName    char   output basename, e.g. 'T1_island_baseline'
%     Pld_MW      double island real load [MW]
%     StopTime_s  double sim stop time [s]  (10 = smoke, 30 = full)
%     isSmoke     logical (optional) label the summary line as SMOKE; for a
%                 smoke the settling thresholds are reported as INFO, not
%                 asserted (10 s is too short to fully settle).
%
% The on-disk .slx is NOT modified (taps are in-memory; closed with 0).
% DOES NOT touch any control/build_*.m code — run/record/save only.
%
% NOTE (2026-07-02 lesson): the discrete EMS-rate signals get their OWN dec=1
% tap; never resample a 0.1 s signal through the 1 ms tap's decimation.

    if nargin < 4 || isempty(isSmoke), isSmoke = (StopTime_s < 20); end
    mdl = 'microgrid_case5';

    %% ── robust project-root resolution ──
    w = which('build_microgrid_eelib');
    if ~isempty(w), here = fileparts(w); else, here = fileparts(mfilename('fullpath')); here = fileparts(here); end
    root = fileparts(here);
    if exist(fullfile(root,'mg_params.mat'),'file') ~= 2
        for cand = {pwd, fileparts(pwd)}
            if exist(fullfile(cand{1},'mg_params.mat'),'file') == 2, root = cand{1}; break; end
        end
    end
    slx = fullfile(root,'SIMULINK',[mdl '.slx']);
    assert(isfile(slx), 'microgrid_case5.slx not found at %s', slx);
    verDir = fullfile(root,'RESULTS','verification');
    if ~exist(verDir,'dir'), mkdir(verDir); end

    %% ── base workspace: params + f0 + build flags ──
    S = load(fullfile(root,'mg_params.mat')); assignin('base','mg', S.mg);
    Hs = load(fullfile(root,'h2_params.mat')); h2s = struct();
    fn = fieldnames(Hs); for i=1:numel(fn), h2s.(fn{i}) = Hs.(fn{i}); end
    assignin('base','h2', h2s);
    assignin('base','f0', 50);
    assignin('base','vsc_warm_start',        true);
    assignin('base','vsc_inner_loop_enable', true);

    %% ── load on-disk model ──
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    load_system(slx);

    %% ── solver + disk-safety ──
    set_param(mdl,'SolverType','Fixed-step'); set_param(mdl,'Solver','ode14x');
    set_param(mdl,'FixedStep','50e-6'); set_param(mdl,'StartTime','0.0');
    set_param(mdl,'StopTime',num2str(StopTime_s)); set_param(mdl,'SimulationMode','normal');
    set_param(mdl,'SaveOutput','off'); set_param(mdl,'SaveTime','off');
    set_param(mdl,'SimscapeLogType','none'); set_param(mdl,'SignalLogging','off');
    set_param(mdl,'DSMLogging','off');
    try, set_param(mdl,'InspectSignalLogs','off'); catch, end

    %% ── island scenario ──
    Qld_MVAr = Pld_MW * tan(acos(0.95));
    set_param([mdl '/Src_CB_PCC_cmd'],'Value','0');
    set_param([mdl '/Src_Pload_MW'],  'Value',num2str(Pld_MW));
    set_param([mdl '/Src_Qload_MVAr'],'Value',num2str(Qld_MVAr,'%.4f'));
    for b = {'SM_Bus1','SM_Bus3','SM_Bus5'}
        set_param([mdl '/TransmissionNetwork/' b{1} '/D_pu'],'Value','1.5');
    end
    fprintf('[vrun] %s: island D_pu=1.5 P=%g MW Q=%.2f MVAr Stop=%g s\n', ...
        caseName, Pld_MW, Qld_MVAr, StopTime_s);

    %% ── taps (electrical dec=20; EMS-native dec=1) ──
    y = 80;
    tapOut (mdl,'DCBus','Out_V_dc_V','V_dc',20,y); y=y+40;
    tapGoto(mdl,'f_meas','f_meas',20,y); y=y+40;
    tapOut (mdl,'ESS','Out_P_ess_MW','P_ess_act',20,y); y=y+40;
    tapOut (mdl,'PEMFC','Out_P_fc_MW','P_fc_act',20,y); y=y+40;
    tapOut (mdl,'Electrolyzer','Out_P_elec_MW','P_elec_act',20,y); y=y+40;
    % ── NEW MEASURED POWER CHANNELS (publication: previously derived, now taps) ──
    % Purely additive To-Workspace sinks on existing module outports; they add NO
    % feedback and CANNOT change the sim dynamics -> battery/FC/etc are unchanged.
    tapOut (mdl,'PV','Out_P_pv_MW','P_pv_act',20,y); y=y+40;               % PV -> DC bus
    tapOut (mdl,'Wind','Out_P_wind_MW','P_wind_act',20,y); y=y+40;         % wind -> DC bus
    tapOut (mdl,'GridFormingVSC','Out_P_vsc_MW','P_vsc_act',20,y); y=y+40; % VSC DC->AC
    tapGoto(mdl,'SOC_ESS','SOC_ESS',20,y); y=y+40;
    tapGoto(mdl,'SOC_H2','SOC_H2',20,y); y=y+40;
    tapOut (mdl,'EMS','Out_Pref_ess_MW','Pref_ess',1,y); y=y+40;
    tapOut (mdl,'EMS','Out_Pcmd_fc_MW','Pcmd_fc',1,y); y=y+40;
    tapOut (mdl,'EMS','Out_Pcmd_elec_MW','Pcmd_elec',1,y); y=y+40;
    tapOut (mdl,'EMS','Out_state','EMS_state',1,y); y=y+40;

    %% ── compile guard ──
    compiled = true;
    try, set_param(mdl,'SimulationCommand','update'); catch ME
        compiled = false; fprintf('*** COMPILE FAILED: %s\n', regexprep(ME.message,'<[^>]*>','')); end
    if ~compiled, close_system(mdl,0); error('vrun_island:compileFailed','%s did not compile',caseName); end

    %% ── simulate ──
    fprintf('[vrun] simulating %g s @50us ode14x (island)...\n', StopTime_s);
    t0=tic; simErr='';
    try, out = sim(mdl); catch ME, simErr = ME.message; end
    wall = toc(t0);
    fprintf('[vrun] wall %.0f s\n', wall);

    %% ── extract ──
    R = struct('caseName',caseName,'Pld_MW',Pld_MW,'Qld_MVAr',Qld_MVAr, ...
               'StopTime_s',StopTime_s,'isSmoke',isSmoke,'wall_s',wall,'simErr',simErr);
    if isempty(simErr)
        g = @(nm) squeeze(out.get(nm).Data);
        R.t_e=out.get('V_dc').Time; R.Vdc=g('V_dc'); R.f=g('f_meas');
        R.Pess=g('P_ess_act'); R.Pfc=g('P_fc_act'); R.Pelec=g('P_elec_act');
        R.socE=g('SOC_ESS'); R.socH=g('SOC_H2');
        R.t_ems=out.get('EMS_state').Time; R.Pref=g('Pref_ess');
        R.PcmdFc=g('Pcmd_fc'); R.PcmdElec=g('Pcmd_elec'); R.State=g('EMS_state');
        R.Ppv=g('P_pv_act'); R.Pwind=g('P_wind_act'); R.Pvsc=g('P_vsc_act');

        % settled window = last 5 s of FINITE data. Robust to an empty/short log
        % or a trailing non-finite sample (last5win guards the empty-find case);
        % mwin averages only finite samples. On clean data this is identical to
        % mean(x(n5:end)).
        [~, win] = last5win(R.t_e);
        mwin = @(x) mean(x(win & isfinite(x(:))));   % finite-only mean over window
        R.Vdc_mean=mwin(R.Vdc); R.f_mean=mwin(R.f);
        R.Pess_end=R.Pess(end); R.Pfc_end=R.Pfc(end);

        % ── NEW: last-5 s steady-state means + measured power-balance loss ──
        % Sign convention: generation & storage-discharge (+), load & consumption
        % subtracted. P_ess>0 = battery discharging, P_elec>0 = electrolyzer load.
        R.Ppv_mean=mwin(R.Ppv);   R.Pwind_mean=mwin(R.Pwind);
        R.Pvsc_mean=mwin(R.Pvsc); R.Pess_mean=mwin(R.Pess);
        R.Pfc_mean=mwin(R.Pfc);   R.Pelec_mean=mwin(R.Pelec);
        R.Pload_MW=Pld_MW;   % island steady load; A8 restores V~1 pu so the
                             % constant-Z load draws ~= its nominal rating.
        % system loss = SUM(generation) + storage-discharge - load - consumption
        R.Ploss_MW=(R.Ppv_mean+R.Pwind_mean+R.Pfc_mean+R.Pess_mean) ...
                   -(R.Pload_MW+R.Pelec_mean);
        % independent cross-check via the measured VSC transfer (DC->AC):
        R.Pdc_inj_MW = R.Ppv_mean+R.Pwind_mean+R.Pfc_mean+R.Pess_mean-R.Pelec_mean;
        R.Lossdc_MW  = R.Pdc_inj_MW - R.Pvsc_mean;   % DC lines + VSC/xfmr conversion
        R.Lossac_MW  = R.Pvsc_mean  - R.Pload_MW;    % AC bus (VSC output -> load)
        R.finiteOK = all(isfinite(R.Vdc)) && all(isfinite(R.f)) && all(isfinite(R.Pess));
        R.chans_ok = ~isempty(R.Vdc) && ~isempty(R.State);
        % physics indicators (asserted only on the full run)
        R.ind_Vdc = abs(R.Vdc_mean-1500) < 20;
        R.ind_f   = abs(R.f_mean-50)     < 0.2;
        R.ind_stable = any(R.State==3);
        R.ind_noEmerg = ~any(R.State==4);

        fprintf('\n── %s settled window (last 5 s) ──\n', caseName);
        fprintf('   V_dc mean %.1f V  | f mean %.4f Hz | P_ess end %.1f | P_fc end %.1f MW\n', ...
            R.Vdc_mean, R.f_mean, R.Pess_end, R.Pfc_end);
        fprintf('   SOC_ESS %.4f->%.4f | SOC_H2 %.4f->%.4f | EMS state end %d\n', ...
            R.socE(1),R.socE(end),R.socH(1),R.socH(end),R.State(end));
        fprintf('   EMS_state trace: %s\n', mat2str(R.State(:).'));
        fprintf('\n── %s MEASURED power balance (last 5 s mean, MW) ──\n', caseName);
        fprintf('   PV %+.1f | Wind %+.1f | Batt %+.1f | FC %+.1f | Elec %+.1f | VSC %+.1f | Load %+.1f\n', ...
            R.Ppv_mean, R.Pwind_mean, R.Pess_mean, R.Pfc_mean, -R.Pelec_mean, R.Pvsc_mean, -R.Pload_MW);
        fprintf('   P_loss (SUM gen+stor - load - cons) = %.2f MW  [DC-side %.2f + AC-side %.2f]\n', ...
            R.Ploss_MW, R.Lossdc_MW, R.Lossac_MW);
        fprintf('   balance residual (gen+stor-load-cons-loss) = %.3f MW (should be ~0)\n', ...
            R.Ploss_MW - (R.Ppv_mean+R.Pwind_mean+R.Pfc_mean+R.Pess_mean-R.Pload_MW-R.Pelec_mean));
    else
        fprintf('*** SIM FAILED: %s\n', regexprep(simErr,'<[^>]*>',''));
        R.finiteOK=false; R.chans_ok=false;
    end

    %% ── plumbing PASS (smoke) vs physics PASS (full) ──
    plumbingPASS = isempty(simErr) && R.chans_ok && R.finiteOK;
    if isSmoke
        R.PASS = plumbingPASS;   verdict = ternstr(R.PASS,'PASS','FAIL');
        note = sprintf('SMOKE %gs: sim+channels+finite=%s | INFO Vdc=%.1f f=%.4f state_end=%d', ...
            StopTime_s, ternstr(plumbingPASS,'ok','BAD'), ...
            getf(R,'Vdc_mean'), getf(R,'f_mean'), getfi(R,'State'));
    else
        R.PASS = plumbingPASS && R.ind_Vdc && R.ind_f && R.ind_stable && R.ind_noEmerg;
        verdict = ternstr(R.PASS,'PASS','FAIL');
        note = sprintf('FULL %gs: Vdc=%.1f(%s) f=%.4f(%s) state3=%s noEMERG=%s Pfc_end=%.1f', ...
            StopTime_s, getf(R,'Vdc_mean'), ynstr(R.ind_Vdc), getf(R,'f_mean'), ynstr(R.ind_f), ...
            ynstr(R.ind_stable), ynstr(R.ind_noEmerg), getf(R,'Pfc_end'));
    end

    %% ── save + summary line + clean close ──
    outMat = fullfile(verDir,[caseName '.mat']);
    save(outMat,'-struct','R');
    R.matfile = outMat;
    line = sprintf('[%s] %-28s %-5s | %s | wall %.0fs\n', ...
        datestr(now,'yyyy-mm-dd HH:MM'), caseName, verdict, note, wall);
    fid = fopen(fullfile(verDir,'SUMMARY.txt'),'a'); fprintf(fid,'%s',line); fclose(fid);
    fprintf('\n[vrun] %s', line);
    fprintf('[vrun] saved %s\n', outMat);
    close_system(mdl,0);
end

%% ── helpers ──
function [n5, win] = last5win(t)
    % Indices of the last-5-s steady-state window, anchored on the last FINITE
    % time sample. Guards against an empty/short log or a trailing non-finite
    % sample (which makes the raw find(t>=t(end)-5,1) return EMPTY). Returns n5
    % (first index in the window) and a logical mask win (same length as t).
    t = t(:); fin = isfinite(t);
    win = false(size(t));
    if any(fin)
        t_end = max(t(fin));
        win = (t >= t_end-5) & fin;
        if ~any(win), win = fin; end
    end
    n5 = find(win, 1);
    if isempty(n5), n5 = 1; end
end
function tapOut(mdl,ssName,portName,wsName,dec,ypos)
    ss=[mdl '/' ssName]; k=str2double(get_param([ss '/' portName],'Port'));
    h=get_param(ss,'PortHandles').Outport(k); tw=[mdl '/TW_' wsName];
    if getSimulinkBlockHandle(tw)>0, return; end
    add_block('simulink/Sinks/To Workspace',tw,'VariableName',wsName,'SaveFormat','Timeseries', ...
        'Decimation',num2str(dec),'Position',[1100 ypos 1180 ypos+24]);
    add_line(mdl,h,get_param(tw,'PortHandles').Inport(1),'autorouting','on');
end
function tapGoto(mdl,tag,wsName,dec,ypos)
    fr=[mdl '/From_' wsName]; tw=[mdl '/TW_' wsName];
    if getSimulinkBlockHandle(tw)>0, return; end
    add_block('simulink/Signal Routing/From',fr,'GotoTag',tag,'Position',[1000 ypos 1060 ypos+24]);
    add_block('simulink/Sinks/To Workspace',tw,'VariableName',wsName,'SaveFormat','Timeseries', ...
        'Decimation',num2str(dec),'Position',[1100 ypos 1180 ypos+24]);
    add_line(mdl,['From_' wsName '/1'],['TW_' wsName '/1'],'autorouting','on');
end
function s=ternstr(c,a,b), if c, s=a; else, s=b; end, end
function s=ynstr(c), if c, s='Y'; else, s='N'; end, end
function v=getf(R,f), if isfield(R,f), v=R.(f); else, v=NaN; end, end
function v=getfi(R,f), if isfield(R,'State')&&~isempty(R.State), v=R.State(end); else, v=-1; end, end
