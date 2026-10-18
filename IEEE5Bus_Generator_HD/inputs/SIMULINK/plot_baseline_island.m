%% plot_baseline_island.m  — PART 0: publication figures from the island baseline
% Reads RESULTS/baseline_island_210.mat (baseline_island_210.m output) and
% produces the "anlamlı plotlar" for the TUBITAK report / paper:
%
%   FIG 1  Island response overview (6 panels): V_dc, f, dispatch (ESS+FC),
%          EMS state, SOC trajectories, power balance.
%   FIG 2  Settling zoom [0..10 s] with ENTSO-E / DC bands + settle markers.
%   FIG 3  Steady-state quality (last 5 s): ripple bands, dispatch split.
%
% Saves PNG (300 dpi) + .fig to RESULTS/figures/.
% Startup energization inrush (t<0.1 s, purely numerical black-start) is drawn
% but annotated; steady metrics use the last-5-s window.
%
% RUN:  >> run('SIMULINK/plot_baseline_island.m')

clear; clc; close all;

% ── locate project + data ------------------------------------------------
here = fileparts(mfilename('fullpath'));
if isempty(here) || ~isfile(fullfile(here,'build_microgrid_eelib.m'))
    w = which('build_microgrid_eelib'); if ~isempty(w), here = fileparts(w); end
end
root = fileparts(here);
matf = fullfile(root,'RESULTS','baseline_island_210.mat');
assert(isfile(matf), 'baseline mat not found: %s\nRun baseline_island_210.m first.', matf);
figdir = fullfile(root,'RESULTS','figures'); if ~isfolder(figdir), mkdir(figdir); end
S = load(matf);

te = S.t_e(:);  tm = S.t_ems(:);
Vdc=S.Vdc(:); f=S.f(:); Pe=S.Pess(:); Pf=S.Pfc(:); sE=S.socE(:); sH=S.socH(:);
St=S.State(:); Pref=S.Pref(:); Pcf=S.PcmdFc(:);

% ── colors (CLAUDE.md convention) ----------------------------------------
cF=[0 0.45 0.74]; cE=[0.20 0.62 0.20]; cR=[0.85 0.10 0.10];
cM=[0.72 0.10 0.72]; cK=[0.15 0.15 0.15]; cGrey=[0.5 0.5 0.5];
lw=1.6;

% ── settle metrics --------------------------------------------------------
tsV = te(find(abs(Vdc-1500)>5 ,1,'last'));    % last time |dVdc|>5 V
tsF = te(find(abs(f-50)  >0.1,1,'last'));      % last time |df|>0.1 Hz
w5  = te>=(te(end)-5);
stat=@(x)[mean(x(w5)) min(x(w5)) max(x(w5))];
Vs=stat(Vdc); Fs=stat(f); Es=stat(Pe); Rs=stat(Pf);

%% ══════════════════ FIG 1 — island response overview ══════════════════
figure('Name','Island baseline overview','Color','w','Position',[60 60 1180 760]);
tl=tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
title(tl,sprintf('Island-mode baseline  —  P_{load}=210 MW, D_{pu}=1.5, GFM VSC (A2/A8), 50 \\mus/ode14x'),...
    'FontWeight','bold');

nexttile; plot(te,Vdc,'Color',cF,'LineWidth',lw); grid on; hold on;
yline(1500,'--','Color',cGrey,'HandleVisibility','off');
yline([1425 1575],':','Color',cR,'HandleVisibility','off');
ylabel('V_{dc} [V]'); title('DC-bus voltage'); ylim([1400 1620]);
xline(tsV,'-.','Color',cE,'Label',sprintf('settle %.1fs',tsV),'HandleVisibility','off');

nexttile; plot(te,f,'Color',cF,'LineWidth',lw); grid on; hold on;
yline(50,'--','Color',cGrey,'HandleVisibility','off');
yline([49.5 50.5],':','Color',cR,'HandleVisibility','off');
ylabel('f [Hz]'); title('System frequency (A8 restoration)'); ylim([49 51]);
xline(tsF,'-.','Color',cE,'Label',sprintf('settle %.1fs',tsF),'HandleVisibility','off');

nexttile; plot(te,Pe,'Color',cE,'LineWidth',lw,'DisplayName','P_{ess} (batt)'); grid on; hold on;
plot(te,Pf,'Color',cR,'LineWidth',lw,'DisplayName','P_{fc}');
stairs(tm,Pref,'--','Color',cE*0.6,'LineWidth',1.0,'DisplayName','P_{ess}^{ref} (EMS)');
stairs(tm,Pcf,'--','Color',cR*0.6,'LineWidth',1.0,'DisplayName','P_{fc}^{cmd} (EMS)');
ylabel('P [MW]'); title('Dispatch: battery + fuel cell'); legend('Location','best','FontSize',7);

nexttile; stairs(tm,St,'Color',cK,'LineWidth',lw); grid on;
yticks(1:4); yticklabels({'GRID','TRANS','STABLE','EMERG'}); ylim([0.5 4.5]);
title('EMS state (Stateflow, A6-L1)'); ylabel('state');

nexttile; plot(te,sE,'Color',cE,'LineWidth',lw,'DisplayName','SOC_{ESS}'); grid on; hold on;
plot(te,sH,'Color',cM,'LineWidth',lw,'DisplayName','SOC_{H2}');
ylabel('SOC [-]'); title('Storage state of charge'); legend('Location','best','FontSize',8);
ylim([0 1]);

nexttile; plot(te,Pe+Pf,'Color',cK,'LineWidth',lw,'DisplayName','P_{ess}+P_{fc}'); grid on; hold on;
yline(mean(Pe(w5)+Pf(w5)),':','Color',cGrey,...
    'Label',sprintf('%.0f MW local supply',mean(Pe(w5)+Pf(w5))),'HandleVisibility','off');
ylabel('P [MW]'); xlabel('t [s]'); title('Local generation (island: PCC open)');
legend('Location','best','FontSize',8);

xlabel(tl,'t [s]');
exportgraphics(gcf,fullfile(figdir,'fig1_island_overview.png'),'Resolution',300);
savefig(gcf,fullfile(figdir,'fig1_island_overview.fig'));

%% ══════════════════ FIG 2 — settling zoom [0..10 s] ══════════════════
figure('Name','Settling zoom','Color','w','Position',[90 90 1000 620]);
tl2=tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
title(tl2,'Black-start settling (island): V_{dc} and f converge to setpoints','FontWeight','bold');

nexttile; plot(te,Vdc,'Color',cF,'LineWidth',lw); grid on; hold on;
yline(1500,'--','Color',cGrey,'Label','1500 V','HandleVisibility','off');
yline([1425 1575],':','Color',cR,'Label','\pm5%','HandleVisibility','off');
xline(tsV,'-.','Color',cE,'Label',sprintf('V_{dc} settled @ %.1fs',tsV),'HandleVisibility','off');
xlim([0 10]); ylim([600 1800]); ylabel('V_{dc} [V]'); title('DC bus');

nexttile; plot(te,f,'Color',cF,'LineWidth',lw); grid on; hold on;
yline(50,'--','Color',cGrey,'Label','50 Hz','HandleVisibility','off');
yline([49.5 50.5],':','Color',cR,'Label','ENTSO-E','HandleVisibility','off');
xline(tsF,'-.','Color',cE,'Label',sprintf('f settled @ %.1fs',tsF),'HandleVisibility','off');
xlim([0 10]); ylim([20 82]); ylabel('f [Hz]'); xlabel('t [s]'); title('Frequency');
text(0.15,30,'energization inrush (t<0.1 s, numerical)','FontSize',7,'Color',cGrey);
exportgraphics(gcf,fullfile(figdir,'fig2_settling_zoom.png'),'Resolution',300);
savefig(gcf,fullfile(figdir,'fig2_settling_zoom.fig'));

%% ══════════════════ FIG 3 — steady-state quality (last 5 s) ══════════════════
figure('Name','Steady-state quality','Color','w','Position',[120 120 1000 420]);
tl3=tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
title(tl3,'Steady-state quality (last 5 s of 30 s run)','FontWeight','bold');

nexttile; hold on; grid on;
plot(te(w5),Vdc(w5),'Color',cF,'LineWidth',lw);
yline(1500,'--','Color',cGrey,'HandleVisibility','off');
ylabel('V_{dc} [V]'); xlabel('t [s]');
title(sprintf('V_{dc}: mean %.2f V, ripple %.2f V pk-pk',Vs(1),Vs(3)-Vs(2)));

nexttile; hold on; grid on;
b=bar([Es(1) Rs(1); NaN NaN]);
b(1).FaceColor=cE; b(2).FaceColor=cR;
xticks(1); xticklabels({''});
legend({sprintf('battery %.1f MW',Es(1)),sprintf('fuel cell %.1f MW',Rs(1))},'Location','northoutside','Orientation','horizontal');
ylabel('P [MW]');
title(sprintf('Local supply %.1f MW (of 210 MW load; renewables + line losses balance)',Es(1)+Rs(1)));
exportgraphics(gcf,fullfile(figdir,'fig3_steady_quality.png'),'Resolution',300);
savefig(gcf,fullfile(figdir,'fig3_steady_quality.fig'));

%% ── console summary ------------------------------------------------------
fprintf('\n=== island baseline figures ===\n');
fprintf('V_dc  last5s: mean %.2f V   [%.1f .. %.1f]   settle %.2f s\n',Vs(1),Vs(2),Vs(3),tsV);
fprintf('f     last5s: mean %.4f Hz  [%.3f .. %.3f]   settle %.2f s\n',Fs(1),Fs(2),Fs(3),tsF);
fprintf('P_ess last5s: %.2f MW    P_fc last5s: %.2f MW    (local %.1f MW)\n',Es(1),Rs(1),Es(1)+Rs(1));
fprintf('EMS end-state: %d (3=STABLE)   EMERGENCY samples: %d\n',St(end),sum(St==4));
fprintf('saved 3 figs (png+fig) -> %s\n',figdir);
