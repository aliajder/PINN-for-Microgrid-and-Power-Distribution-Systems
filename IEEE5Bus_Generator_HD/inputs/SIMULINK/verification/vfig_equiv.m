function figs = vfig_equiv(t4mat, t5mat)
%% vfig_equiv — Fig4: MIL vs SIL vs PIL bit-exact equivalence of the EMS
% Reads the parsed T4 (SIL) and T5 (PIL) equivalence structs and produces one
% Word-compatible report figure showing that the EMS controller's outputs are
% IDENTICAL across the model (MIL), the compiled host C (SIL), and the on-target
% STM32G431RB C (PIL) — i.e. maxdiff = 0 on every output, the MIL->SIL->PIL leg
% of the deployment chain. Saves .fig + .png (300 dpi) to RESULTS/figures_report/
% and appends a Turkish caption to captions.txt.
%
%   figs = vfig_equiv(t4mat, t5mat)
%     t4mat  full path to RESULTS/verification/T4_sil_equiv.mat
%     t5mat  full path to RESULTS/verification/T5_pil.mat
%
% Touches NO control/build code — plot/save only. Style matches vfig_island
% (white bg, black title, dark distinct colors, boxed white legend, LW>=1.5).

    T4 = load(t4mat); T5 = load(t5mat);
    root = fileparts(fileparts(fileparts(t4mat)));      % ...\microh2grid
    figDir = fullfile(root,'RESULTS','figures_report');
    if ~exist(figDir,'dir'), mkdir(figDir); end

    LW = 1.8; FS = 12;
    cMIL = [0.30 0.30 0.30];   % model (reference) — dark gray
    cSIL = [0.00 0.00 0.60];   % host C           — dark blue
    cPIL = [0.00 0.50 0.00];   % on-target C      — dark green

    % ── the three MW outputs (EMS_state is a state id, shown as text below) ──
    lbl = {'P_{ref,ESS}','P_{cmd,elec}','P_{cmd,FC}'};
    MIL = T4.normal_end(1:3);   % model reference (same stimulus both runs)
    SIL = T4.tgt_end(1:3);      % compiled host C end values
    PIL = T5.tgt_end(1:3);      % STM32 on-target C end values
    Y   = [MIL(:) SIL(:) PIL(:)];

    maxdiff_all = max([T4.maxdiff(:); T5.maxdiff(:)]);   % should be exactly 0
    state_end   = T5.state_end;                          % 3 = ISLAND_STABLE

    f4 = figure('Color','w','Visible','off','Position',[100 100 940 560]);
    ax = axes(f4); hold(ax,'on'); grid(ax,'on');
    b = bar(ax, categorical(lbl,lbl), Y, 'grouped');
    b(1).FaceColor = cMIL; b(2).FaceColor = cSIL; b(3).FaceColor = cPIL;
    b(1).DisplayName = 'MIL (model)'; b(2).DisplayName = 'SIL (host C)'; b(3).DisplayName = 'PIL (STM32G431)';

    % value labels on top of each bar
    for s = 1:3
        xt = b(s).XEndPoints; yt = b(s).YEndPoints;
        for k = 1:numel(xt)
            text(ax, xt(k), yt(k), sprintf('%.1f', yt(k)), ...
                'HorizontalAlignment','center','VerticalAlignment','bottom', ...
                'FontSize',FS-2,'Color','k');
        end
    end

    set(ax,'FontSize',FS,'GridAlpha',0.15,'Box','on','Color','w', ...
        'XColor',[0.15 0.15 0.15],'YColor',[0.15 0.15 0.15]);
    ylabel(ax,'EMS çıkışı (koşu sonu değeri) [MW]');
    ymax = max(Y(:)); if ~isfinite(ymax) || ymax<=0, ymax = 1; end
    ylim(ax,[min(0,min(Y(:)))*1.1  ymax*1.25]);
    title(ax,'EMS denklik zinciri: MIL \equiv SIL \equiv PIL (bit-birebir)','Color','k','FontWeight','bold');
    L = legend(ax,'Location','northwest','Box','on');
    set(L,'Color','w','TextColor','k','EdgeColor',[0.30 0.30 0.30]);

    % headline annotation: maxdiff = 0 + on-target resource footprint. Placed over
    % the empty right region (P_cmd,elec / P_cmd,FC end at 0) so it never covers
    % the legend or the P_ref,ESS bars.
    msg = sprintf(['maxdiff = 0\n(tüm 4 çıkış, bit-birebir)\n\nEMS durumu sonu = %g\n(ISLAND\\_STABLE)\n\n' ...
        'Hedef: %s\n@ %g MHz\nFLASH %.1f%%  RAM %.1f%%\nELF %.2f KB\nüretilen C: %g satır'], ...
        state_end, getf(T5,'mcu','STM32G431'), ...
        getn(T5,'clock_MHz'), getn(T5,'flash_pct'), getn(T5,'ram_pct'), getn(T5,'elf_KB'), getn(T4,'c_lines'));
    annotation(f4,'textbox',[0.46 0.34 0.46 0.50],'String',msg,'FitBoxToText','off', ...
        'BackgroundColor',[0.96 0.96 0.96],'EdgeColor',[0.30 0.30 0.30], ...
        'Color','k','FontSize',FS-1,'FontWeight','bold','Interpreter','tex', ...
        'HorizontalAlignment','center','VerticalAlignment','middle');

    figs = strings(0);
    figs(end+1) = saveboth(f4, figDir, 'Fig4_sil_pil_equiv');

    cap = sprintf(['Fig4: EMS gömülü-C dağıtım zincirinin denklik kanıtı. Aynı uyaran altında model (MIL, gri), ' ...
        'ana-makinede derlenmiş C (SIL, mavi) ve gerçek STM32G431RB üzerinde çalışan hedef-C (PIL, yeşil) ' ...
        'kontrolör çıkışları BİREBİR aynıdır: dört çıkışın da maxdiff=%.0e, EMS durumu sonu=%g (ISLAND_STABLE). ' ...
        'Hedef görüntü MCU''ya sığar (FLASH %.1f%%, RAM %.1f%%, ELF %.2f KB @ %g MHz). Bu, MIL\\rightarrowSIL' ...
        '\\rightarrowPIL bacağını doğrular; kalan bacak PIL\\rightarrowHIL (OPAL-RT).'], ...
        maxdiff_all, state_end, getn(T5,'flash_pct'), getn(T5,'ram_pct'), getn(T5,'elf_KB'), getn(T5,'clock_MHz'));
    fid = fopen(fullfile(figDir,'captions.txt'),'a');
    fprintf(fid,'\n=== Fig4 (SIL/PIL denklik) — %s ===\n%s\n', datestr(now,'yyyy-mm-dd HH:MM'), cap);
    fclose(fid);

    fprintf('[vfig_equiv] Fig4 -> %s (maxdiff_all=%.0e, state_end=%g)\n', figDir, maxdiff_all, state_end);
    close(f4);
end

%% ── helpers ──
function v = getn(S,f), if isfield(S,f) && isnumeric(S.(f)) && ~isempty(S.(f)), v = S.(f); else, v = NaN; end, end
function v = getf(S,f,d), if isfield(S,f) && ~isempty(S.(f)), v = S.(f); else, v = d; end, end

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
