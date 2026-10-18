%% build_ems_pil.m — EMS Processor-in-the-Loop on STM32G431RB (ready-to-run)
% Twin of build_ems_sil.m, but runs the compiled EMS on the REAL STM32G431RB
% (Nucleo-64) over serial. PIL executes the exact C that is flashed to the MCU
% and co-simulates it against the (host-side) scenario, so it verifies the
% controller on target hardware — the final embedded-readiness step for the EMS.
%
% ── STATUS on this machine (probed 2026-07-06) ──────────────────────────────
% DONE : MATLAB-side G4xx target IS installed (stm32setup G4xx download).
%        HardwareBoard 'STM32G4xx Based' (Cortex-M4) + board NUCLEO-G431RB are
%        REGISTERED; toolchain 'GNU Tools for STM32' is registered.
% TOOLCHAIN (re-probed 2026-07-19 — an earlier "tools are missing" note was
%        WRONG; it only searched C:\Program Files, and everything lives on E:):
%          STM32CubeMX            E:\STM32CubeMX                  ✓
%          STM32CubeIDE 2.1.1     E:\ST\STM32CubeIDE_2.1.1        ✓
%            └ STM32_Programmer_CLI.exe (flashing)                ✓
%          arm-none-eabi-gcc      installed BY the MATLAB stm32b package into
%            C:\ProgramData\MATLAB\R2026a\stm32b\3P.instrset\... ✓
%          .ioc  C:\stm32\ems_g431\ems_g431.ioc  → STM32G431RBTx, 170 MHz,
%                USART2 enabled (ST-Link VCP), toolchain Makefile ✓
% REMAINING WORK — must be done in the STM32CubeMX GUI, then "Generate Code".
% Do NOT text-edit the .ioc: MATLAB drives CubeMX headlessly during the build and
% CubeMX REWRITES the .ioc, silently dropping settings that a hand edit left
% inconsistent. Observed 2026-07-19: regex-editing Toolchain+NoMain made CubeMX
% rewrite the file and DELETE the whole USART2 peripheral (Mcu.IP4, IPParameters,
% VM_ASYNC, MX_USART2_UART_Init) — i.e. it removed the very serial link PIL needs.
% Restored from ems_g431.ioc.bak. Open the GUI and set, in this order:
%   1. Project Manager > Project  > Toolchain/IDE        = STM32CubeIDE
%        (blockset supports ONLY MDK-ARM or STM32CubeIDE — see iocParser.m:285;
%         'Makefile' makes it build the class name stm32cube.makefile, which
%         does not exist → "Unable to resolve the name 'stm32cube.makefile'")
%   2. Project Manager > Code Generator > "Do not generate the main()" = CHECKED
%        (else stm32:codegen:STM32CubeMXNoMainNotEnabled)
%   3. System Core > SYS > Timebase Source = TIM6  (anything but SysTick —
%        Simulink needs SysTick for task scheduling)
%        (else stm32:codegen:STM32CubeMXTimebaseSourceSystick)
%   4. VERIFY Connectivity > USART2 = Asynchronous survived (PIL serial link,
%        routed to the Nucleo ST-Link VCP on PA2/PA3).
%   5. Generate Code, then re-run this script.
%
% TWO REAL TRAPS, both fixed below:
%   1. The .ioc must sit on a SPACE-FREE path (see iocFile).
%   2. 'STM32CubeMXProjectFile' is NOT a set_param property — the .ioc must be
%      written into the codertarget data tree (see the block further down).
%      Getting this wrong is what produced the long-standing
%      stm32:codegen:NoSTM32CubeMXProjectFile failure.
%
% RUN (after the package is installed + BOARD set):
%   >> run('SIMULINK/build_ems_pil.m')

clear; clc;
% BOARD confirmed registered on this machine (stm32setup G4xx install):
%   HardwareBoard = 'STM32G4xx Based' (Cortex-M4), board NUCLEO-G431RB.
% NOTE: still needs the EXTERNAL ST toolchain on disk (STM32CubeMX + GNU ARM
% gcc + STM32CubeProgrammer). If absent, the build errors telling you which.
BOARD = 'STM32G4xx Based';

mdl = 'ems_pil';
ph  = @(b) get_param(b,'PortHandles');

% same host-side hardening/space-path fixes as build_ems_sil.m
if ~isempty(getenv('NoDefaultCurrentDirectoryInExePath'))
    setenv('NoDefaultCurrentDirectoryInExePath','');
end
cgRoot = 'C:\mw_codegen'; if ~isfolder(cgRoot), mkdir(cgRoot); end
Simulink.fileGenControl('set','CodeGenFolder',cgRoot,'CacheFolder',cgRoot,'createDir',true);
cd(cgRoot);

if isempty(BOARD)
    error(['STM32 HardwareBoard not set. Install the STM32 support package, then set ' ...
           'BOARD to the exact registered name (see this file''s header). ' ...
           'PIL cannot target the STM32G431RB until then.']);
end

% ── build the same EMS I/O model as build_ems_sil.m ─────────────────────────
if bdIsLoaded(mdl), close_system(mdl,0); end
new_system(mdl);
set_param(mdl,'Solver','FixedStepDiscrete','SolverType','Fixed-step','FixedStep','0.1');
build_ems(mdl);
glob = {'f_meas','SOC_ESS','SOC_H2','CB_PCC'}; y=40;
for k=1:numel(glob)
    ip=[mdl '/IN_' glob{k}]; gt=[mdl '/GT_' glob{k}];
    add_block('simulink/Sources/In1',ip,'Position',[20 y 50 y+14]);
    add_block('simulink/Signal Routing/Goto',gt,'GotoTag',glob{k},'TagVisibility','global','Position',[90 y 150 y+14]);
    add_line(mdl,ph(ip).Outport(1),ph(gt).Inport(1),'autorouting','on'); y=y+30;
end
pw={'P_pv',1;'P_wind',2;'P_load',3};
for k=1:size(pw,1)
    ip=[mdl '/IN_' pw{k,1}]; add_block('simulink/Sources/In1',ip,'Position',[20 y 50 y+14]);
    add_line(mdl,ph(ip).Outport(1),ph([mdl '/EMS']).Inport(pw{k,2}),'autorouting','on'); y=y+30;
end
outs={'Pref_ess',1;'Pcmd_elec',2;'Pcmd_fc',3;'EMS_state',4}; y=40;
for k=1:size(outs,1)
    op=[mdl '/OUT_' outs{k,1}]; add_block('simulink/Sinks/Out1',op,'Position',[480 y 510 y+14]);
    add_line(mdl,ph([mdl '/EMS']).Outport(outs{k,2}),ph(op).Inport(1),'autorouting','on'); y=y+30;
end

% ── target config for the STM32 (ERT + STM32 board + PIL) ───────────────────
cs=getActiveConfigSet(mdl);
set_param(cs,'SystemTargetFile','ert.tlc','TargetLang','C');
set_param(cs,'HardwareBoard',BOARD);                       % registered STM32 board

% ── point the freshly-created 'ems_pil' model at the STM32CubeMX .ioc project ──
% (build_ems_pil creates a NEW model via new_system, so any .ioc set in the GUI
%  on another model does NOT carry over — it must be set here explicitly.)
% SPACE-FREE PATH IS MANDATORY. Pointing this at the CubeMX project under the
% OneDrive tree ("OneDrive - HAKAN SARI - PRUVA YEMINLI ...") is ACCEPTED by
% set_param but then fails at build time with
%   stm32:codegen:NoSTM32CubeMXProjectFile
% because the STM32 blockset's CubeMX/Makefile flow cannot handle spaces in the
% path (same class of failure as the codegen folder → C:\mw_codegen).
% Keep a mirrored copy at C:\stm32\ems_g431 and point here.
iocFile = 'C:\stm32\ems_g431\ems_g431.ioc';
assert(isfile(iocFile), ['ioc bulunamadi: %s\n' ...
    'CubeMX projesini bosluksuz bir yola kopyala, orn:\n' ...
    '  copyfile(''...\\Belgeler\\STM32\\ems_g431'', ''C:\\stm32\\ems_g431'')'], iocFile);
% ⚠ NOT a set_param property. `set_param(cs,'STM32CubeMXProjectFile',...)` errors
% with "Property does not exist" — the .ioc lives in the codertarget data tree.
% The build reads it via stm32cube.utils.getCubeParams(cs).ProjectFile, which maps
% to the codertarget parameter 'STM32CubeMX.ProjectFile' (getCubeParams.m:53).
% Leaving it unset leaves the default placeholder '<Select STM32CubeMX project
% file>' → the build dies in stm32cube/iocParser.m:36 with
% stm32:codegen:NoSTM32CubeMXProjectFile.
% This mirrors what the GUI's stm32cube.parameters.setCubeMXProjectDetails does.
iocObj = stm32cube.iocParser(iocFile);
mcu    = iocObj.getMicrocontrollerInfo;
codertarget.data.setParameterValue(cs,'STM32CubeMX.ProjectFile', iocFile);
codertarget.data.setParameterValue(cs,'STM32CubeMX.DeviceId',    mcu.Name);
codertarget.data.setParameterValue(cs,'STM32CubeMX.Family',      mcu.Family);
codertarget.data.setParameterValue(cs,'Clocking', ...
    struct('cpuClockRateMHz', getSystemClockInformation(iocObj)));
% PIL talks over the Nucleo's ST-Link virtual COM port; USART2 is the .ioc default.
uartPort = stm32cube.parameters.internal.getDefaultUARTForConnectivity(iocObj);
if ~isempty(uartPort)
    codertarget.data.setParameterValue(cs,'Connection.SerialModule', uartPort);
end
% PIN THE HOST COM PORT. 'Connection.SerialModule' above is the MCU-side
% PERIPHERAL (USART2) — it does NOT say which Windows COM port the host opens.
% Left unset, the target auto-picks the first port it enumerates: it grabbed COM3
% and failed with SelectedPortBusy, while this Nucleo's ST-Link Virtual COM Port
% is COM11. Parameter name is 'Connection.Serialport' (lowercase 'p') — written
% by stm32cube's own callbacks and read back in
% +stm32cube/+parameters/serialPortValueCallback.m:13.
% If the board is ever replugged to a different port, change PIL_COM_PORT only.
PIL_COM_PORT = 'COM11';   % Nucleo-G431RB ST-Link Virtual COM Port (Device Manager)
codertarget.data.setParameterValue(cs,'Connection.Serialport', PIL_COM_PORT);

% Round-trip check: fail loudly HERE rather than deep inside the build.
iocStored = stm32cube.utils.getCubeParams(cs).ProjectFile;
fprintf('   .ioc  : %s (isfile=%d)\n', iocStored, isfile(iocStored));
fprintf('   MCU   : %s (%s), %g MHz, PIL UART=%s\n', ...
    mcu.Name, mcu.Family, getSystemClockInformation(iocObj), string(uartPort));
comStored = codertarget.data.getParameterValue(cs,'Connection.Serialport');
fprintf('   COM   : %s (istenen %s)\n', string(comStored), PIL_COM_PORT);
assert(strcmpi(comStored, PIL_COM_PORT), ...
    'PIL COM portu yazilamadi: istenen %s, model %s', PIL_COM_PORT, string(comStored));
assert(isfile(iocStored), 'ioc yolu modele yazilamadi: %s', iocStored);
set_param(cs,'CodeExecutionProfiling','on');               % on-target timing (RAM/exec)
set_param(cs,'CodeProfilingInstrumentation','coarse');

% ── scenario (same island deficit as SIL) + Normal-vs-PIL equivalence ───────
dt=0.1; Tend=12; t=(0:dt:Tend)'; mk=@(v) timeseries(v*ones(size(t)),t);
sig.f_meas  = timeseries([50*ones(sum(t<5),1);49*ones(sum(t>=5&t<8),1);50*ones(sum(t>=8),1)],t);
sig.SOC_ESS=mk(0.50); sig.SOC_H2=mk(0.50);
sig.CB_PCC = timeseries([ones(sum(t<5),1);zeros(sum(t>=5),1)],t);
sig.P_pv=mk(80); sig.P_wind=mk(80); sig.P_load=mk(200);
order={'f_meas','SOC_ESS','SOC_H2','CB_PCC','P_pv','P_wind','P_load'};
ds=Simulink.SimulationData.Dataset; for k=1:numel(order), ds=ds.addElement(sig.(order{k}),order{k}); end
assignin('base','ds',ds);
set_param(mdl,'LoadExternalInput','on','ExternalInput','ds','StopTime',num2str(Tend), ...
    'SaveOutput','on','OutputSaveName','yout','SaveFormat','Dataset');

fprintf('=== EMS PIL on %s ===\n', BOARD);
set_param(mdl,'SimulationMode','normal');                        oN=sim(mdl);
set_param(mdl,'SimulationMode','Processor-in-the-loop (PIL)');   oS=sim(mdl);  % flashes + co-sims
labels={'Pref_ess','Pcmd_elec','Pcmd_fc','EMS_state'}; getsig=@(o,i) squeeze(o.yout{i}.Values.Data);
mism=0; fprintf('\n-- Normal vs PIL (max abs diff) --\n');
for i=1:numel(labels)
    a=getsig(oN,i); b=getsig(oS,i); d=max(abs(a(:)-b(:)));
    fprintf('   %-10s end N=%9.3f  PIL=%9.3f  maxdiff=%.3e\n',labels{i},a(end),b(end),d);
    mism=mism+(d>1e-6);
end
fprintf('\nPIL_EQUIV (on-target C == model): %s\n', string(mism==0));
set_param(mdl,'SimulationMode','normal'); close_system(mdl,0);