function R = vrun_island_step(caseName, Plo_MW, Phi_MW, t_step_s, StopTime_s, isSmoke, ramp_time_s)
%% vrun_island_step — island verification with a MID-SIM LOAD RAMP (T3)
% Same engine as vrun_island (island: CB_PCC=0, physical D_pu=1.5, 50 us /
% ode14x, SDI+Simscape logging OFF, own-rate taps), but the island real load
% goes Plo_MW -> Phi_MW starting at t = t_step_s, RAMPED over ramp_time_s. This
% exercises the hydrogen dispatch handover (battery fast response -> fuel cell
% slow take-over, FCpdon trigger) under a genuine transient rather than a static
% load.
%
%   R = vrun_island_step(caseName, Plo_MW, Phi_MW, t_step_s, StopTime_s, isSmoke, ramp_time_s)
%     caseName    char   output basename, e.g. 'T3_h2_dispatch'
%     Plo_MW      double load before the step [MW]  (e.g. 160)
%     Phi_MW      double load after  the step [MW]  (e.g. 210)
%     t_step_s    double step instant [s]           (e.g. 15)
%     StopTime_s  double sim stop time [s]          (10 = smoke, 30 = full)
%     isSmoke     logical (optional) label the summary line as SMOKE
%     ramp_time_s double (optional) load-ramp duration [s], default 1.5
%
% MECHANISM (in-memory harness surgery — NO control/build_*.m code touched):
% the master wires Src_Pload_MW (Constant) as a BRANCH to BUS/In_P_load_MW and
% EMS/In_P_load_MW, and Src_Qload_MVAr (Constant) to BUS/In_Q_load_MVAr
% (wire_microgrid.m G4/G5). Here we DELETE those two Constants and drop in a
% Step (Before=lo, After=hi, Time=t_step) FOLLOWED BY A RATE LIMITER, wired to
% the SAME destinations. The rate limiter ramps the level change over
% ramp_time_s so the constant-Z load never sees an unphysical 50 us conductance
% impulse — an ideal 50 us step produced a SPURIOUS f-nadir (~22 Hz) that
% wrongly tripped EMS EMERGENCY even though the system is actually stable
% (project rate-limiter lesson). The rate limiter's InitialCondition = lo so
% there is NO 0->lo ramp at t=0; only the t=t_step change is limited. P and Q
% share the SAME ramp duration, so pf 0.95 lagging holds throughout. The on-disk
% .slx is NOT modified (closed with 0).
%
% NOTE (2026-07-02 lesson): discrete EMS-rate signals get their OWN dec=1 tap;
% never resample a 0.1 s signal through the 1 ms electrical tap's decimation.

    if nargin < 6 || isempty(isSmoke),     isSmoke     = (StopTime_s < 20); end
    if nargin < 7 || isempty(ramp_time_s), ramp_time_s = 1.5; end
    mdl = 'microgrid_case5';
    pf  = 0.95;
    Qlo = Plo_MW * tan(acos(pf));
    Qhi = Phi_MW * tan(acos(pf));

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

    %% ── island scenario: open PCC, D_pu=1.5 ──
    set_param([mdl '/Src_CB_PCC_cmd'],'Value','0');
    for b = {'SM_Bus1','SM_Bus3','SM_Bus5'}
        set_param([mdl '/TransmissionNetwork/' b{1} '/D_pu'],'Value','1.5');
    end

    %% ── LOAD-RAMP INJECTION: swap the two load Constants for Step+RateLimiter ──
    slewP = replaceConstWithRamp(mdl, 'Src_Pload_MW',   Plo_MW, Phi_MW, t_step_s, ramp_time_s);
    slewQ = replaceConstWithRamp(mdl, 'Src_Qload_MVAr', Qlo,    Qhi,    t_step_s, ramp_time_s);
    fprintf(['[vrun_step] %s: island D_pu=1.5 | P %g->%g MW (%.1f MW/s), ' ...
             'Q %.2f->%.2f MVAr (%.1f MVAr/s) ramp @ t=%g s over %.2f s | Stop=%g s\n'], ...
        caseName, Plo_MW, Phi_MW, slewP, Qlo, Qhi, slewQ, t_step_s, ramp_time_s, StopTime_s);

    %% ── taps (electrical dec=20; EMS-native dec=1) ──
    y = 80;
    tapOut (mdl,'DCBus','Out_V_dc_V','V_dc',20,y); y=y+40;
    tapGoto(mdl,'f_meas','f_meas',20,y); y=y+40;
    tapOut (mdl,'ESS','Out_P_ess_MW','P_ess_act',20,y); y=y+40;
    tapOut (mdl,'PEMFC','Out_P_fc_MW','P_fc_act',20,y); y=y+40;
    tapOut (mdl,'Electrolyzer','Out_P_elec_MW','P_elec_act',20,y); y=y+40;
    % ── NEW MEASURED POWER CHANNELS (publication: previously derived, now taps) ──
    % Purely additive To-Workspace sinks on existing module outports; add NO
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
    if ~compiled, close_system(mdl,0); error('vrun_island_step:compileFailed','%s did not compile',caseName); end

    %% ── simulate ──
    fprintf('[vrun_step] simulating %g s @50us ode14x (island, load step)...\n', StopTime_s);
    t0=tic; simErr='';
    try, out = sim(mdl); catch ME, simErr = ME.message; end
    wall = toc(t0);
    fprintf('[vrun_step] wall %.0f s\n', wall);

    %% ── extract ──
    R = struct('caseName',caseName,'Plo_MW',Plo_MW,'Phi_MW',Phi_MW,'t_step_s',t_step_s, ...
               'Qlo_MVAr',Qlo,'Qhi_MVAr',Qhi,'StopTime_s',StopTime_s,'isSmoke',isSmoke, ...
               'wall_s',wall,'simErr',simErr);
    if isempty(simErr)
        g = @(nm) squeeze(out.get(nm).Data);
        R.t_e=out.get('V_dc').Time; R.Vdc=g('V_dc'); R.f=g('f_meas');
        R.Pess=g('P_ess_act'); R.Pfc=g('P_fc_act'); R.Pelec=g('P_elec_act');
        R.socE=g('SOC_ESS'); R.socH=g('SOC_H2');
        R.t_ems=out.get('EMS_state').Time; R.Pref=g('Pref_ess');
        R.PcmdFc=g('Pcmd_fc'); R.PcmdElec=g('Pcmd_elec'); R.State=g('EMS_state');
        R.Ppv=g('P_pv_act'); R.Pwind=g('P_wind_act'); R.Pvsc=g('P_vsc_act');

        % settled window = last 5 s of FINITE data (post-step steady state).
        % Robust to an empty/short log or a trailing non-finite sample: T3 is the
        % borderline scenario and if the run diverges the raw find(t>=t(end)-5,1)
        % returns EMPTY (t(end) NaN) -> n5=[] -> indexing crash. last5win guards
        % it so extraction always RECORDS the result instead of dying, and mwin
        % averages only finite samples so a trailing NaN cannot poison the means.
        % On clean data this is identical to mean(x(n5:end)).
        [~, win] = last5win(R.t_e);
        mwin = @(x) mean(x(win & isfinite(x(:))));   % finite-only mean over window
        R.Vdc_mean=mwin(R.Vdc); R.f_mean=mwin(R.f);
        R.Pess_end=R.Pess(end); R.Pfc_end=R.Pfc(end);

        % ── NEW: last-5 s (post-step) means + measured power-balance loss ──
        % Load here is the POST-STEP steady load (Phi_MW). A8 restores V~1 pu so
        % the constant-Z load draws ~= its nominal rating.
        R.Ppv_mean=mwin(R.Ppv);   R.Pwind_mean=mwin(R.Pwind);
        R.Pvsc_mean=mwin(R.Pvsc); R.Pess_mean=mwin(R.Pess);
        R.Pfc_mean=mwin(R.Pfc);   R.Pelec_mean=mwin(R.Pelec);
        R.Pload_MW=Phi_MW;   % post-step steady load
        R.Ploss_MW=(R.Ppv_mean+R.Pwind_mean+R.Pfc_mean+R.Pess_mean) ...
                   -(R.Pload_MW+R.Pelec_mean);
        R.Pdc_inj_MW = R.Ppv_mean+R.Pwind_mean+R.Pfc_mean+R.Pess_mean-R.Pelec_mean;
        R.Lossdc_MW  = R.Pdc_inj_MW - R.Pvsc_mean;   % DC lines + VSC/xfmr conversion
        R.Lossac_MW  = R.Pvsc_mean  - R.Pload_MW;    % AC bus (VSC output -> load)
        % pre-step window = last 2 s before the step (baseline handover reference)
        ipre = find(R.t_e < t_step_s, 1, 'last');
        ip0  = find(R.t_e >= t_step_s-2, 1);
        if ~isempty(ipre) && ~isempty(ip0)
            R.Pess_prestep = mean(R.Pess(ip0:ipre));
            R.Pfc_prestep  = mean(R.Pfc(ip0:ipre));
            R.f_nadir_poststep = min(R.f(R.t_e >= t_step_s));
        else
            R.Pess_prestep = NaN; R.Pfc_prestep = NaN; R.f_nadir_poststep = NaN;
        end
        R.finiteOK = all(isfinite(R.Vdc)) && all(isfinite(R.f)) && all(isfinite(R.Pess));
        R.chans_ok = ~isempty(R.Vdc) && ~isempty(R.State);
        R.ind_Vdc = abs(R.Vdc_mean-1500) < 20;
        R.ind_f   = abs(R.f_mean-50)     < 0.2;
        R.ind_stable = any(R.State==3);
        R.ind_noEmerg = ~any(R.State==4);

        fprintf('\n── %s settled window (last 5 s) ──\n', caseName);
        fprintf('   V_dc mean %.1f V | f mean %.4f Hz | f nadir post-step %.4f Hz\n', ...
            R.Vdc_mean, R.f_mean, R.f_nadir_poststep);
        fprintf('   P_ess pre-step %.1f -> end %.1f MW | P_fc pre-step %.1f -> end %.1f MW\n', ...
            R.Pess_prestep, R.Pess_end, R.Pfc_prestep, R.Pfc_end);
        fprintf('   SOC_ESS %.4f->%.4f | SOC_H2 %.4f->%.4f | EMS state end %d\n', ...
            R.socE(1),R.socE(end),R.socH(1),R.socH(end),R.State(end));
        fprintf('   EMS_state trace: %s\n', mat2str(R.State(:).'));
        fprintf('\n── %s MEASURED power balance (last 5 s post-step mean, MW) ──\n', caseName);
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
        note = sprintf('SMOKE %gs step %g->%g@%gs: sim+chan+finite=%s | INFO Vdc=%.1f f=%.4f state_end=%d', ...
            StopTime_s, Plo_MW, Phi_MW, t_step_s, ternstr(plumbingPASS,'ok','BAD'), ...
            getf(R,'Vdc_mean'), getf(R,'f_mean'), getfi(R));
    else
        R.PASS = plumbingPASS && R.ind_Vdc && R.ind_f && R.ind_stable && R.ind_noEmerg;
        verdict = ternstr(R.PASS,'PASS','FAIL');
        note = sprintf('FULL %gs step %g->%g@%gs: Vdc=%.1f(%s) f=%.4f(%s) state3=%s noEMERG=%s Pfc_end=%.1f', ...
            StopTime_s, Plo_MW, Phi_MW, t_step_s, getf(R,'Vdc_mean'), ynstr(R.ind_Vdc), ...
            getf(R,'f_mean'), ynstr(R.ind_f), ynstr(R.ind_stable), ynstr(R.ind_noEmerg), getf(R,'Pfc_end'));
    end

    %% ── save + summary line + clean close ──
    outMat = fullfile(verDir,[caseName '.mat']);
    save(outMat,'-struct','R');
    R.matfile = outMat;
    line = sprintf('[%s] %-28s %-5s | %s | wall %.0fs\n', ...
        datestr(now,'yyyy-mm-dd HH:MM'), caseName, verdict, note, wall);
    fid = fopen(fullfile(verDir,'SUMMARY.txt'),'a'); fprintf(fid,'%s',line); fclose(fid);
    fprintf('\n[vrun_step] %s', line);
    fprintf('[vrun_step] saved %s\n', outMat);
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
function slew = replaceConstWithRamp(mdl, blkName, before, after, tstep, ramp_time)
    % Replace top-level Constant blkName with a Step -> Rate Limiter chain wired
    % to the SAME destination port(s). Preserves the branch (Const -> N dst).
    % The rate limiter ramps the before->after change over ramp_time (slew =
    % |after-before|/ramp_time); its InitialCondition = before so there is NO
    % 0->before ramp at t=0. Returns the slew rate [units/s] for logging.
    blk = [mdl '/' blkName];
    assert(getSimulinkBlockHandle(blk) > 0, 'block %s not found', blkName);
    pos = get_param(blk,'Position');
    lh  = get_param(blk,'LineHandles');
    dst = get_param(lh.Outport(1),'DstPortHandle');   % array if branched
    dst = dst(dst > 0);
    % delete the outport line(s) EXPLICITLY first — delete_block leaves dangling
    % half-lines on the destination ports of a branched line, which then blocks
    % add_line ("destination port already has a line connection").
    for L = lh.Outport(:).'
        if L > 0 && ishandle(L), delete_line(L); end
    end
    delete_block(blk);
    % Step (ideal set-point) at the original Constant location
    add_block('simulink/Sources/Step', blk, ...
        'Time',   num2str(tstep), ...
        'Before', num2str(before), ...
        'After',  num2str(after), ...
        'SampleTime','0', ...
        'Position', pos);
    % Rate Limiter just downstream — ramps the level change over ramp_time so the
    % constant-Z load never sees an unphysical 50 us conductance impulse.
    slew = abs(after - before) / ramp_time;
    rl   = [mdl '/RL_' erase(blkName,'Src_')];        % e.g. RL_Pload_MW
    rlpos = [pos(1)+80 pos(2) pos(3)+80 pos(4)];
    add_block('simulink/Discontinuities/Rate Limiter', rl, ...
        'RisingSlewLimit',  num2str( slew), ...
        'FallingSlewLimit', num2str(-slew), ...
        'InitialCondition', num2str(before), ...
        'Position', rlpos);
    add_line(mdl, get_param(blk,'PortHandles').Outport(1), ...
                  get_param(rl,'PortHandles').Inport(1), 'autorouting','on');
    rlout = get_param(rl,'PortHandles').Outport(1);
    for k = 1:numel(dst)
        add_line(mdl, rlout, dst(k), 'autorouting','on');
    end
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
function v=getfi(R), if isfield(R,'State')&&~isempty(R.State), v=R.State(end); else, v=-1; end, end
