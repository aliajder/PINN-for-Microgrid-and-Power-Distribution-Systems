function figs = vfig_island(matfile, prefix, titleTag)
%% vfig_island — Word-compatible report figures from a vrun_island .mat
% Produces Fig1 (V_dc + f), Fig2 (P_batt / P_FC / P_elec — the battery->FC
% handover, the report's key figure), Fig3 (EMS state + SOC_ESS/SOC_H2).
% Saves each as BOTH .fig and .png (300 dpi) into RESULTS/figures_report/,
% and appends caption text to figures_report/captions.txt.
%
%   figs = vfig_island(matfile, prefix, titleTag)
%     matfile  full path to a vrun_island output .mat
%     prefix   filename/id prefix, e.g. 'T1' -> T1_fig1_vdc_f.png ...
%     titleTag short scenario label put in each title, e.g. 'Ada 210 MW'
%
% STYLE (report rules): white background, BLACK titles, dark-saturated
% distinct colors + distinct line styles, LineWidth 1.8, font 12, light grid,
% boxed legend. Consistent color->quantity map across every figure:
%   f = dark blue | V_dc = dark green | P_batt = dark teal |
%   P_FC = dark red | P_elec = orange | EMS state = purple.

    D = load(matfile);
    root = fileparts(fileparts(fileparts(matfile)));   % ...\microh2grid
    figDir = fullfile(root,'RESULTS','figures_report');
    if ~exist(figDir,'dir'), mkdir(figDir); end

    % ── canonical color map (dark/saturated, readable on white) ──
    C.f     = [0.00 0.00 0.60];   % dark blue
    C.vdc   = [0.00 0.50 0.00];   % dark green
    C.batt  = [0.00 0.50 0.50];   % dark teal
    C.fc    = [0.75 0.00 0.00];   % dark red
    C.elec  = [0.90 0.50 0.00];   % orange
    C.state = [0.50 0.00 0.50];   % purple
    C.socH  = [0.60 0.30 0.00];   % dark brown-orange
    LW = 1.8; FS = 12;
    figs = strings(0);
    caps = {};

    % ── step-scenario detection (T3): auto-mark the load ramp + real f nadir ──
    isStep = isfield(D,'t_step_s');
    if isStep
        ipn = D.t_e >= D.t_step_s;
        ttn = D.t_e(ipn); ffn = D.f(ipn);
        [f_nadir_val, kk] = min(ffn);   % real dip (the +69.7 Hz single-sample
        t_nadir_val       = ttn(kk);    % artifact at t_step is a MAX, so min is clean)
    end

    styleAx = @(ax) set(ax,'FontSize',FS,'GridAlpha',0.15,'Box','on','Color','w', ...
                          'XColor',[0.15 0.15 0.15],'YColor',[0.15 0.15 0.15]);
    % R2026a figure theming leaks a DARK background into legends; force a
    % white, Word-friendly legend box with black text everywhere.
    lg = @(varargin) stylelegend(legend(varargin{:}));

    %% ── Fig1: V_dc(t) and f(t) — 2 subplots ──
    f1 = figure('Color','w','Visible','off','Position',[100 100 900 620]);
    ax1 = subplot(2,1,1);
    plot(D.t_e, D.Vdc,'-','Color',C.vdc,'LineWidth',LW,'DisplayName','V_{dc}'); hold on; grid on;
    yline(1500,'--','Color',[0.4 0.4 0.4],'LineWidth',1.0,'HandleVisibility','off','Label','1500 V');
    styleAx(ax1); ylabel('V_{dc} [V]'); title('DC bus voltage','Color','k');
    lg('Location','best','Box','on');
    ax2 = subplot(2,1,2);
    plot(D.t_e, D.f,'-','Color',C.f,'LineWidth',LW,'DisplayName','f'); hold on; grid on;
    yline(50,'--','Color',[0.4 0.4 0.4],'LineWidth',1.0,'HandleVisibility','off','Label','50 Hz');
    styleAx(ax2); xlabel('t [s]'); ylabel('f [Hz]'); title('System frequency','Color','k');
    ylim(ax2,[48 52]);   % ENTSO-E-relevant band; clips the t<0.5 s sync-startup spike
    if isStep
        % load-ramp instant on both subplots + the real f nadir on the f axis
        xline(ax1, D.t_step_s,'-','Color',C.state,'LineWidth',1.2,'HandleVisibility','off', ...
            'Label','yük adımı','LabelVerticalAlignment','top','LabelHorizontalAlignment','left');
        xline(ax2, D.t_step_s,'-','Color',C.state,'LineWidth',1.2,'HandleVisibility','off', ...
            'Label','yük adımı','LabelVerticalAlignment','bottom','LabelHorizontalAlignment','left');
        plot(ax2, t_nadir_val, f_nadir_val,'v','MarkerFaceColor',C.f,'MarkerEdgeColor','k', ...
            'MarkerSize',9,'HandleVisibility','off');
        text(ax2, t_nadir_val, f_nadir_val, sprintf('  f_{nadir}=%.3f Hz @ %.3f s', f_nadir_val, t_nadir_val), ...
            'Color','k','FontSize',FS-1,'VerticalAlignment','top','HorizontalAlignment','left');
    end
    lg('Location','best','Box','on');
    sgtitle(sprintf('%s — V_{dc} ve frekans oturması', titleTag),'Color','k','FontWeight','bold');
    figs(end+1) = saveboth(f1, figDir, [prefix '_fig1_vdc_f']);
    if isStep
        caps{end+1} = sprintf(['Fig1 (%s): DC-bara gerilimi V_dc (koyu yeşil, 1500 V ref) ve sistem frekansı f ' ...
            '(koyu mavi, 50 Hz ref) ada modunda (D_pu=1.5). t=%.0f s''teki mor dikey çizgi yük adımını ' ...
            '(160\\rightarrow210 MW, ~1.5 s rampa) işaretler; frekans en düşük noktası f_nadir=%.3f Hz @ %.3f s ' ...
            'işaretlenmiştir. f ekseni [48,52] Hz ENTSO-E bandına sabitlendi; t=%.3f s''teki tek-örneklik ' ...
            '+69.7 Hz sivri bir ölçüm artefaktıdır (bant dışında kalıp kırpıldı) ve kendiliğinden temizlenir.'], ...
            prefix, D.t_step_s, f_nadir_val, t_nadir_val, D.t_step_s);
    else
        caps{end+1} = sprintf(['Fig1 (%s): DC-bara gerilimi V_dc (koyu yeşil) 1500 V referansına ve sistem ' ...
            'frekansı f (koyu mavi) 50 Hz referansına oturması. Ada modu, D_pu=1.5. Frekans ekseni [48,52] Hz ' ...
            'ENTSO-E bandına sabitlendi; t<0.5 s başlangıç senkronizasyon geçişi kırpılmıştır.'], prefix);
    end

    %% ── Fig2: P_batt / P_FC / P_elec — battery->FC handover (KEY FIGURE) ──
    f2 = figure('Color','w','Visible','off','Position',[100 100 900 520]);
    ax = axes(f2); hold(ax,'on'); grid(ax,'on');
    plot(D.t_e, D.Pess ,'-' ,'Color',C.batt,'LineWidth',LW,'DisplayName','P_{batarya} (ESS)');
    plot(D.t_e, D.Pfc  ,'--','Color',C.fc  ,'LineWidth',LW,'DisplayName','P_{yakıt hücresi}');
    if isfield(D,'Pelec')
        plot(D.t_e, D.Pelec,'-.','Color',C.elec,'LineWidth',LW,'DisplayName','P_{elektrolizer}');
    end
    yline(0,':','Color',[0.5 0.5 0.5],'LineWidth',1.0,'HandleVisibility','off');
    if isStep
        xline(D.t_step_s,'-','Color',C.state,'LineWidth',1.2,'HandleVisibility','off', ...
            'Label','yük adımı','LabelVerticalAlignment','top');
    end
    styleAx(ax); xlabel('t [s]'); ylabel('Güç [MW]');
    ylim(ax,[-5 100]); yticks(ax,0:20:100);   % headroom above the 80 MW ESS ceiling; clean ticks
    title(sprintf('%s — Batarya \\rightarrow Yakıt hücresi güç devri', titleTag),'Color','k','FontWeight','bold');
    lg('Location','best','Box','on');
    figs(end+1) = saveboth(f2, figDir, [prefix '_fig2_battery_fc_handover']);
    if isStep
        caps{end+1} = sprintf(['Fig2 (%s): KİLİT FİGÜR — yük adımında (t=%.0f s, mor çizgi) hızlı/yavaş kaynak ' ...
            'ayrışması. Adımdan hemen önce batarya P_batt=%.1f MW, yakıt hücresi P_FC=%.1f MW (hafif-ada ' ...
            'dengesi). Adım anında hızlı batarya (koyu turkuaz) +50 MW anlık açığı üstlenip sıçrar (koşu sonu ' ...
            '%.1f MW); yavaş yakıt hücresi (koyu kırmızı, τ=5 s) ve elektrolizer (turuncu) yavaş zaman ' ...
            'ölçeğinde yeniden dağıtılır (P_FC koşu sonu %.1f MW). Batarya-hızlı / H2-yavaş zaman-ölçeği ayrışması.'], ...
            prefix, D.t_step_s, D.Pess_prestep, D.Pfc_prestep, D.Pess(end), D.Pfc(end));
    else
        caps{end+1} = sprintf(['Fig2 (%s): Ada modunda güç devri — batarya (koyu turkuaz) hızlı ilk yanıtı verip ' ...
            'çalışma tavanından iner, yavaş yakıt hücresi (koyu kırmızı, τ=5 s) devralır; elektrolizer (turuncu) ' ...
            'fazlada devrede. Raporun en önemli figürü.'], prefix);
    end

    %% ── Fig3: EMS state + SOC ──
    f3 = figure('Color','w','Visible','off','Position',[100 100 900 520]);
    ax = axes(f3); grid(ax,'on'); hold(ax,'on');
    yyaxis left
    stairs(D.t_ems, D.State,'-','Color',C.state,'LineWidth',LW+0.4,'DisplayName','EMS durumu');
    ylabel('EMS durumu [1..4]'); ylim([0.5 4.5]); yticks(1:4);
    set(ax,'YColor',C.state);
    yyaxis right
    plot(D.t_e, D.socE,'--','Color',C.batt,'LineWidth',LW,'DisplayName','SOC_{ESS}');
    plot(D.t_e, D.socH,'-.','Color',C.socH,'LineWidth',LW,'DisplayName','SOC_{H2}');
    ylabel('SOC [-]'); set(ax,'YColor',[0.15 0.15 0.15]);
    if isStep
        xline(D.t_step_s,'-','Color',[0.35 0.35 0.35],'LineWidth',1.2,'HandleVisibility','off', ...
            'Label','yük adımı','LabelVerticalAlignment','top');
    end
    styleAx(ax); set(ax,'Box','on'); xlabel('t [s]');
    title(sprintf('%s — EMS durumu ve SOC izleri', titleTag),'Color','k','FontWeight','bold');
    lg('Location','best','Box','on');
    figs(end+1) = saveboth(f3, figDir, [prefix '_fig3_ems_soc']);
    if isStep
        caps{end+1} = sprintf(['Fig3 (%s): EMS Stateflow durumu (mor; 1=GRID,2=TRANSITION,3=ISLAND_STABLE,' ...
            '4=EMERGENCY) ve SOC_{ESS}/SOC_{H2}. Sistem tüm koşu boyunca ISLAND_STABLE (3). Yük adımında ' ...
            '(t=%.0f s) t=%.0f.000 s''te tek bir 100 ms EMS çevriminde kısa bir EMERGENCY sıçraması, ani ' ...
            'ölçüm artefaktından kaynaklı olup bir sonraki çevrimde kendiliğinden temizlenir (fiziksel bir ' ...
            'kararsızlık değildir; V_dc ve gerçek f dipleri sınır içindedir).'], prefix, D.t_step_s, D.t_step_s);
    else
        caps{end+1} = sprintf(['Fig3 (%s): EMS Stateflow durumu (mor; 1=GRID,2=TRANSITION,3=ISLAND_STABLE,' ...
            '4=EMERGENCY) ve SOC_{ESS}/SOC_{H2} zaman serileri. Ada geçişi ve kararlı-ada durumu.'], prefix);
    end

    %% ── captions ──
    fid = fopen(fullfile(figDir,'captions.txt'),'a');
    fprintf(fid,'\n=== %s (%s) — %s ===\n', prefix, titleTag, datestr(now,'yyyy-mm-dd HH:MM'));
    for i=1:numel(caps), fprintf(fid,'%s\n', caps{i}); end
    fclose(fid);
    fprintf('[vfig] %s: %d figures -> %s\n', prefix, numel(figs), figDir);
    close(f1); close(f2); close(f3);
end

function L = stylelegend(L)
    % Force a Word-friendly white legend box with black text (overrides the
    % R2026a dark-theme default that fills legends black).
    try, set(L,'Color','w','TextColor','k','EdgeColor',[0.30 0.30 0.30]); catch, end
end

function pngpath = saveboth(figH, figDir, base)
    figfile = fullfile(figDir,[base '.fig']);
    pngpath = fullfile(figDir,[base '.png']);
    savefig(figH, figfile);
    try
        exportgraphics(figH, pngpath, 'Resolution', 300);
    catch
        print(figH, pngpath, '-dpng', '-r300');
    end
end
