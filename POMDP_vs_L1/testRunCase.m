close all;
clear;

name = 'Test';

params = struct('SOAR_VSPEED',0.7, 'SOAR_POMD_ENABLE',0.0,'WP_LOITER_RAD',20.0,'NAVL1_PERIOD',10.0);

rerun=false;

if rerun
    Log = runCase(name, params);
else
    Log = load([name,'.mat']);
end

thermal.pos = [-180,-260];
thermal.w = 4.0;
thermal.R = 80.0;

Log = NormaliseTimes(Log);

% endTimes = [50,300];

% Log = TrimTime(Log, endTimes);

Log = AddSoaringData(Log, thermal);

PlotLogs(Log,'NKF1',{'@-PD'},[],name,'Alt [m]');

h=[];
figure,hold on
colors=get(gca,'ColorOrder');


h(end+1)=plot3(Log.NKF1.PE, Log.NKF1.PN,-Log.NKF1.PD, '-');

plot3(Log.SOAR.estPosE, Log.SOAR.estPosN,   Log.SOAR.alt, ':',...
      Log.SOAR.estPosE, Log.SOAR.estPosN,   Log.SOAR.alt(1) * ones(size(Log.SOAR.alt)), ':');

zl = get(gca,'ZLim');
h(end+1)=plot3([-260,-260],[-180,-180],zl,'ro');
                          
legend(h,[name,'Thermal'],'Interpreter','none');

xlabel('East [m]'); ylabel('North [m]');
axis equal;


h=PlotLogs(Log,'CTUN',{'NavRoll','Roll'},[],name);
legend(h(1:2:end-1),name);

PlotLogs(Log,'SOAR','posErr',[],name);

% True and estimated strength.
PlotLogs(Log,'SOAR','x0',[],name,'Strength [m/s]');
hold on; plot(get(gca,'XLim'), thermal.w*[1,1],'k-');
title('Strength');

% True and estimated rad.
PlotLogs(Log,'SOAR','x1',[],name,'Radius [m]');
hold on; plot(get(gca,'XLim'), thermal.R*[1,1],'k-');
title('Radius');

title('Vario');
leg={};
for iL=1:length(Log)
   leg = [leg,'True', 'Measured', '-vD'];
end
legend(leg);

PlotLogs(Log,'SOAR','dist',[],name);
grid on; grid minor;

poly.A = -0.030993;
poly.B =  0.447319;
poly.C = -2.302930;

v0=9;
sink = (poly.A*v0^2 + poly.B * v0+poly.C)./cosd(Log.CTUN.Roll);

figure;
plot(Log.CTUN.Time,-sink)

% Circling distance to estimate
figure,hold on;
plot(Log.SOAR.Time, sqrt((Log.SOAR.estPosE-Log.SOAR.posE).^2 + (Log.SOAR.estPosN-Log(1).SOAR.posN).^2))
xlabel('Time [s]'); ylabel('Circling radius [m]');
grid on; grid minor;
