close all;

names = {'POMDP_on'};

b = 1.88;
S = 0.45;

params = {struct('SOAR_VSPEED',0.7,...
                'SOAR_POMD_ENABLE',1.0,...
                 'SOAR_POMD_HORI',10.0,...
                 'SOAR_POMD_LOOP',100000,...
                 'SOAR_POMD_N_ACT',10,...
                 'SOAR_POMD_STEP',0.0,...
                 'SOAR_POMD_I_MOM',1.0,...
                 'SOAR_POMD_K_AIL',  25   * b*S*0.25,...
                 'SOAR_POMD_RLLCLP', 0.025 * -b^2*S*1.0,...
                 'SOAR_POMD_RLLDMP', 1.0)};

% params = {struct('SOAR_VSPEED',0.7,...
%                 'SOAR_POMD_ENABLE',1.0,...
%                  'SOAR_POMD_HORI',10.0,...
%                  'SOAR_POMD_LOOP',100000,...
%                  'SOAR_POMD_N_ACT',10,...
%                  'SOAR_POMD_STEP',0.0,...
%                  'SOAR_POMD_I_MOM',0.002575,...
%                  'SOAR_POMD_K_AIL',1.4483,...
%                  'SOAR_POMD_RLLCLP',-1.128,...
%                  'SOAR_POMD_RLLDMP',0.4)};
% rerun=true;
rerun=false;

for iL=1:length(names)
    if rerun
        delete([names{iL},'.mat']);
        runCase(names{iL}, params{iL});
    end
end

clear Logs;
for iL=1:length(names)
    wait=false;
    while ~exist([names{iL},'.mat'],'file')
        pause(1);
        wait=true;
    end
    if wait
        pause(1);
    end
    Logs(iL) = load([names{iL},'.mat']);
end

thermal.pos = [-180,-260];
thermal.w = 4.0;
thermal.R = 80.0;

Logs = NormaliseTimes(Logs);

endTimes = [100,300];

Logs = TrimLog(Logs, endTimes);

Logs = AddSoaringData(Logs, thermal);

h=PlotLogs(Logs,'CTUN',{'NavRoll'},[],names);
legend(h(1:2:end-1),names);
hold on;
plot(Logs.CTUN.Time, Logs.CTUN.Roll,'g');

% And the plan at each time
for i=1:length(Logs.PDBG.Time)
    
    plot(Logs.PDBG.Time(i) + [0:5]*0.2, [Logs.PDBG.theta0(i),Logs.PDBG.theta1(i),Logs.PDBG.theta2(i),Logs.PDBG.theta3(i),Logs.PDBG.theta4(i),Logs.PDBG.theta5(i)],'r');
end

legend('NavRoll','Roll','Plan');
ylabel('Roll [deg]');

figure,plot(Logs.PIDR.Time,Logs.PIDR.I)