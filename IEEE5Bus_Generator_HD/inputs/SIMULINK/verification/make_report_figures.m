%% make_report_figures — Bölüm C: build ALL final report figures in one shot
% Regenerates the Word-compatible report figure set from the verification .mat
% files (T1/T2/T3 island runs + T4/T5 SIL/PIL equivalence). Cleans out the old
% smoke figures and the stale captions.txt so RESULTS/figures_report/ ends up
% with ONLY the final assets + a fresh Turkish captions.txt.
%
% Run AFTER T1..T5 have populated RESULTS/verification/. Touches NO control/
% build code — figures only.
%
%   >> run(fullfile(root,'SIMULINK','verification','make_report_figures.m'))
%
% Produces (each as .fig + .png @300 dpi):
%   T1_fig1_vdc_f / T1_fig2_battery_fc_handover / T1_fig3_ems_soc   (Ada 210 MW)
%   T2_...                                                          (Ada 160 MW hafif)
%   T3_...  (+ yük adımı ve f_nadir işaretli)                       (Ada adım 160->210)
%   Fig4_sil_pil_equiv                                             (MIL=SIL=PIL)
%   captions.txt                                                   (tüm figürler, TR)

    here = fileparts(mfilename('fullpath'));            % ...\SIMULINK\verification
    root = fileparts(fileparts(here));                  % ...\microh2grid
    verDir = fullfile(root,'RESULTS','verification');
    figDir = fullfile(root,'RESULTS','figures_report');
    if ~exist(figDir,'dir'), mkdir(figDir); end
    addpath(here);

    % ── clean previous figure assets (regenerable) so the folder is canonical ──
    for pat = {'*.png','*.fig'}
        d = dir(fullfile(figDir,pat{1}));
        for i=1:numel(d), delete(fullfile(figDir,d(i).name)); end
    end
    capfile = fullfile(figDir,'captions.txt');
    if isfile(capfile), delete(capfile); end
    fid = fopen(capfile,'w');
    fprintf(fid,['FIGÜR AÇIKLAMALARI — Hidrojen Entegre Mikroşebeke HIL Doğrulama\n' ...
                 'Üretim: %s\nRenk haritası: f=koyu mavi, V_dc=koyu yeşil, P_batarya=turkuaz, ' ...
                 'P_YH=koyu kırmızı, P_elektrolizer=turuncu, EMS durumu=mor.\n'], ...
                 datestr(now,'yyyy-mm-dd HH:MM'));
    fclose(fid);

    % ── island runs -> Fig1/2/3 each ──
    jobs = { 'T1_island_baseline', 'T1', 'Ada 210 MW (temel senaryo)'
             'T2_island_light',    'T2', 'Ada 160 MW (hafif yük)'
             'T3_h2_dispatch',     'T3', 'Ada yük adımı 160\rightarrow210 MW' };
    for j = 1:size(jobs,1)
        mat = fullfile(verDir,[jobs{j,1} '.mat']);
        if isfile(mat)
            fprintf('[make_report] %s -> %s\n', jobs{j,1}, jobs{j,2});
            vfig_island(mat, jobs{j,2}, jobs{j,3});
        else
            fprintf('[make_report] SKIP %s (mat yok: %s)\n', jobs{j,1}, mat);
        end
    end

    % ── Fig4: MIL/SIL/PIL equivalence ──
    t4 = fullfile(verDir,'T4_sil_equiv.mat'); t5 = fullfile(verDir,'T5_pil.mat');
    if isfile(t4) && isfile(t5)
        fprintf('[make_report] Fig4 SIL/PIL denklik\n');
        vfig_equiv(t4, t5);
    else
        fprintf('[make_report] SKIP Fig4 (T4/T5 mat eksik)\n');
    end

    d = dir(fullfile(figDir,'*.png'));
    fprintf('\n[make_report] DONE — %d PNG (+ eşdeğer .fig) -> %s\n', numel(d), figDir);
    fprintf('[make_report] captions: %s\n', capfile);
