function dyn = ieee14_dynamic_params()
%IEEE14_DYNAMIC_PARAMS  IEEE 14 baralı modifiye test sistemi dinamik verileri.
%   Kaynak: P. Demetriou, M. Asprou, J. Quiros-Tortos, E. Kyriakides,
%   "Dynamic IEEE Test Systems for Transient Analysis", IEEE Systems Journal,
%   vol. 11, no. 4, 2017 (Tablo I-III). PDF: ieee14bus/Dynamic_IEEE_Test_Systems_for_Transient_Analysis.pdf
%
%   Makineler idealtransformatörle yeni "ünite baralarına" bağlanır:
%     varsayılan bara -> yeni ünite barası : 1->15, 2->16, 3->17, 6->19, 8->18
%   Ünite bara gerilimleri: 15 = 22 kV, 16/17/18/19 = 13.8 kV.
%   Sönüm D, GENROU "machine load damping coefficient" (pu): D*(w-1) biçimi.
%
%   dyn.machine : 5 makine (sıra: ünite 15, 16, 17, 19, 18)
%   dyn.exciter : IEEET1 (Tablo II), dyn.governor : BPA_GG (Tablo III, yalnızca 15 ve 16)
unitBus  = [15 16 17 19 18];
sysBus   = [ 1  2  3  6  8];
% Tablo I sütunları: [1(15), 2(16), 3(17), 6(19) & 8(18) ortak sütun]
col = [1 2 3 4 4];
T1 = struct();
T1.type     = {'GENROU','GENROU','GENROU','GENROU'};
T1.operation= {'gen','gen','cond','cond'};
T1.S_MVA    = [448 100 40 25];
T1.kV       = [22 13.8 13.8 13.8];
T1.H        = [2.656 4.985 1.520 1.200];
T1.D        = [2.000 2.000 0.000 0.000];
T1.ra       = [0.0043 0.0035 0.000 0.0025];
T1.xd       = [1.670 1.180 2.373 1.769];
T1.xq       = [1.600 1.050 1.172 0.855];
T1.xdp      = [0.265 0.220 0.343 0.304];
T1.xqp      = [0.460 0.380 1.172 0.5795];
T1.xdpp     = [0.205 0.145 0.231 0.2035];
T1.xqpp     = [0.205 0.145 0.231 0.2035];
T1.xl       = [0.150 0.075 0.132 0.1045];
T1.Td0p     = [0.5871 1.100 11.600 8.000];
T1.Tq0p     = [0.1351 0.1086 0.159 0.008];
T1.Td0pp    = [0.0248 0.0277 0.058 0.0525];
T1.Tq0pp    = [0.0267 0.0351 0.201 0.0151];
T1.S10      = [0.091 0.0933 0.295 0.304];
T1.S12      = [0.400 0.4044 0.776 0.666];

fn = {'S_MVA','kV','H','D','ra','xd','xq','xdp','xqp','xdpp','xqpp','xl', ...
      'Td0p','Tq0p','Td0pp','Tq0pp','S10','S12'};
m = struct('unitBus', num2cell(unitBus), 'sysBus', num2cell(sysBus));
for k = 1:5
    m(k).type = T1.type{col(k)};
    m(k).operation = T1.operation{col(k)};
    for i = 1:numel(fn)
        m(k).(fn{i}) = T1.(fn{i})(col(k));
    end
end
dyn.machine = m;

T2.S_MVA = [448 100 40 25];
T2.kV    = [22 13.8 13.8 13.8];
T2.Tr    = [0 0.060 0 0];
T2.Ka    = [50 25 400 400];
T2.Ta    = [0.060 0.200 0.050 0.050];
T2.VRmax = [1.000 1.000 6.630 4.407];
T2.VRmin = [-1.000 -1.000 -6.630 -4.407];
T2.Ke    = [-0.0465 -0.0582 -0.170 -0.170];
T2.Te    = [0.520 0.6544 0.950 0.950];
T2.Kf    = [0.0832 0.105 0.040 0.040];
T2.Tf    = [1.000 0.350 1.000 1.000];
T2.E1    = [3.240 2.5785 6.375 4.2375];
T2.SE1   = [0.072 0.0889 0.2174 0.2174];
T2.E2    = [4.320 3.438 8.500 5.650];
T2.SE2   = [0.2821 0.3468 0.9388 0.9386];
T2.type  = 'IEEET1';
dyn.exciter = T2;
dyn.exciter.unitBus = unitBus;
dyn.exciter.column  = col;

T3.type  = 'BPA_GG';
T3.unitBus = [15 16];
T3.S_MVA = [448 100];
T3.kV    = [22 13.8];
T3.Pmax  = [0.870 1.050];
T3.R     = [0.011 0.050];
T3.T1    = [0.100 0.090];
T3.T2    = [0.000 0.000];
T3.T3    = [0.300 0.200];
T3.T4    = [0.050 0.300];
T3.T5    = [10.000 0.000];
T3.F     = [0.250 1.000];
dyn.governor = T3;

dyn.source = 'Demetriou et al., IEEE Systems J. 11(4), 2017, Tables I-III';
end
