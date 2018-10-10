close all;
clear Logs;

names = {'POMDP_off_10m','POMDP_off_15m','POMDP_on_2s','POMDP_on_3s','POMDP_on_4s'};
% names = {'POMDP_off_weak','POMDP_on1_weak','POMDP_on2_weak'};
params = {struct('SOAR_VSPEED',0.7, 'SOAR_POMD_ENABLE',0.0,'WP_LOITER_RAD',10.0,'NAVL1_PERIOD',10.0),...
          struct('SOAR_VSPEED',0.7, 'SOAR_POMD_ENABLE',0.0,'WP_LOITER_RAD',15.0,'NAVL1_PERIOD',10.0),...
          struct('SOAR_VSPEED',0.7, 'SOAR_POMD_ENABLE',1.0,'SOAR_POMD_HORI',2.0,'SOAR_POMD_LOOP',100000,'SOAR_POMD_N_ACT',10,'SOAR_POMD_EXT',3.0),...
          struct('SOAR_VSPEED',0.7, 'SOAR_POMD_ENABLE',1.0,'SOAR_POMD_HORI',3.0,'SOAR_POMD_LOOP',100000,'SOAR_POMD_N_ACT',10,'SOAR_POMD_EXT',3.0),...
          struct('SOAR_VSPEED',0.7, 'SOAR_POMD_ENABLE',1.0,'SOAR_POMD_HORI',4.0,'SOAR_POMD_LOOP',100000,'SOAR_POMD_N_ACT',10,'SOAR_POMD_EXT',3.0)};

% Variants.
if (3)
    nV=3;
    for iL=1:length(names)
        for iV=1:nV
            names2{iL,iV} = [names{iL},'_',num2str(iV)];
            params2{iL,iV}= params{iL};
        end
    end
    names = names2(:);
    params = params2(:);
    [names,idx] = sort(names);
    params = params(idx);
end

% names = {'R20','R30'};
% params = {struct('SOAR_POMD_ENABLE',0.0,'WP_LOITER_RAD',20.0),...
%           struct('SOAR_POMD_ENABLE',0.0,'WP_LOITER_RAD',30.0)};
      
rerun=true;
% rerun=false;
runMask = [0,1,0,0,0];
% runMask = ones(1,5);

for iL=1:length(names)
    if rerun && runMask(iL)
        delete([names{iL},'.mat']);
        delete([names{iL},'.BIN']);
        log=runCase(names{iL}, params{iL});
        Logs(iL) = log;
    else
        Logs(iL) = load([names{iL},'.mat']);
    end
end

thermal.pos = [-180,-260];
thermal.w = 4.0;
thermal.R = 80.0;


Logs = NormaliseTimes(Logs);

endTimes = [100,300];

Logs = TrimLog(Logs, endTimes);

Logs = AddSoaringData(Logs, thermal);

PlotLogs(Logs,'NKF1',{'@-PD'},[],names,'Alt [m]');

h=[];
figure,hold on
colors=get(gca,'ColorOrder');
for iL=1:length(Logs)
    h(end+1)=plot3(Logs(iL).NKF1.PE, Logs(iL).NKF1.PN,-Logs(iL).NKF1.PD, '-','Color',colors(iL,:));

    plot3(Logs(iL).SOAR.estPosE, Logs(iL).SOAR.estPosN,   Logs(iL).SOAR.alt, ':',...
          Logs(iL).SOAR.estPosE, Logs(iL).SOAR.estPosN,   Logs(iL).SOAR.alt(1) * ones(size(Logs(iL).SOAR.alt)), ':','Color',colors(iL,:));
end
zl = get(gca,'ZLim');
h(end+1)=plot3([-260,-260],[-180,-180],zl,'ro');
                          
legend(h,[names,'Thermal'],'Interpreter','none');

xlabel('East [m]'); ylabel('North [m]');
axis equal;


h=PlotLogs(Logs,'CTUN',{'NavRoll','Roll'},[],names);
legend(h(1:2:end-1),names);

PlotLogs(Logs,'SOAR','posErr',[],names);

% True and estimated strength.
PlotLogs(Logs,'SOAR','x0',[],names,'Strength [m/s]');
hold on; plot(get(gca,'XLim'), thermal.w*[1,1],'k-');
title('Strength');

% True and estimated rad.
PlotLogs(Logs,'SOAR','x1',[],names,'Radius [m]');
hold on; plot(get(gca,'XLim'), thermal.R*[1,1],'k-');
title('Radius');

% True and estimated vertical velocities.
PlotLogs(Logs,{'SIM2','SOAR','SIM2'},{'wD','nettorate','@-vD'},[],names,'Rate [m/s]')
hold on;

title('Vario');
leg={};
for iL=1:length(Logs)
   leg = [leg,[names{1},' - true'], [names{1},' - measured'], [names{1},' - -vD']];
end
legend(leg);

PlotLogs(Logs,'SOAR','dist',[],names);

% Compare sink rates
idx=2;
figure,plot(Logs(idx).SIM2.Time, -Logs(idx).SIM2.vD,...
            Logs(idx).SIM2.Time,  Logs(idx).SIM2.wD)
        
Logs(idx).SIM2.Sink = Logs(idx).SIM2.wD+Logs(idx).SIM2.vD;
hold on; plot(Logs(idx).SIM2.Time, Logs(idx).SIM2.Sink)

poly.A = -0.030993;
poly.B =  0.447319;
poly.C = -2.302930;

v0=9;
sink = (poly.A*v0^2 + poly.B * v0+poly.C)./cosd(Logs(idx).CTUN.Roll);

plot(Logs(idx).CTUN.Time,-sink)

% Circling distance to estimate
figure,hold on;
plot(Logs(1).SOAR.Time, sqrt((Logs(1).SOAR.estPosE-Logs(1).SOAR.posE).^2 + (Logs(1).SOAR.estPosN-Logs(1).SOAR.posN).^2))
plot(Logs(2).SOAR.Time, sqrt((Logs(2).SOAR.estPosE-Logs(2).SOAR.posE).^2 + (Logs(2).SOAR.estPosN-Logs(2).SOAR.posN).^2))
xlabel('Time [s]'); ylabel('Circling radius [m]');


PlotLogs(Logs,'AETR','Elev');
