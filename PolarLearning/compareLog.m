close all;
addpath('~/ardupilog');
addpath('~/SoaringLogsAnalysis/');

LogData =  loadLog();

constantK = 49.6;%71.09;

figure,plot(LogData.PLRN.aspd,-LogData.PLRN.z,'r.')

polarTrue.B   = 0.045;
polarTrue.Cd0 = 0.050;
polarTrue.k   = constantK;


polarInit.B   = LogData.PLRN.B(1);
polarInit.Cd0 = LogData.PLRN.CD0(1);
polarInit.k   = constantK;


polarFinal.B   = LogData.PLRN.B(end);
polarFinal.Cd0 = LogData.PLRN.CD0(end);
polarFinal.k   = constantK;

V = 6:0.1:18;

hold on; 
plot(V, GetSinkSpeed(V, polarInit), 'b-');
plot(V, GetSinkSpeed(V, polarFinal),'k-');

plot(V, GetSinkSpeed(V, polarTrue),'r--');

legend('Data','Init','Final','True');

Pinit = [0.005, 0.005];
xinit = [polarInit.Cd0; polarInit.B];
Q = [0;0];
R = 0.5;

ekf = ...
    ExtendedKalmanFilter_polar2state( diag(Pinit.^2),...
                        xinit,...
                        diag(Q.^2),...
                        diag(R.^2));
                    
X = zeros(length(LogData.PLRN.TimeS),2);
P = zeros(length(LogData.PLRN.TimeS),4);


for iT=1:length(LogData.PLRN.TimeS)
    
    FilterMeasurement = -LogData.PLRN.z(iT);
    FiltInputs = [LogData.PLRN.aspd(iT);LogData.PLRN.roll(iT);constantK];
    
    ekf.update(FilterMeasurement,FiltInputs);
    
    X(iT,:) = ekf.x;
    P(iT,:) = ekf.P(:);
end

polarFinalMAtlab.Cd0 = X(end,1);
polarFinalMAtlab.B   = X(end,2);
polarFinalMAtlab.k   = constantK;

plot(V, GetSinkSpeed(V, polarFinalMAtlab),'m--');
legend('Data','Init','Final','True','MATLAB');

grid on; grid minor; 
xlabel('Vx [m/s]');
ylabel('Vz [m/s]');

figure; hold on;
plot(LogData.PLRN.TimeS, LogData.PLRN.CD0,LogData.PLRN.TimeS, LogData.PLRN.B);
hold on; set(gca,'ColorOrderIndex',1);
plot(LogData.PLRN.TimeS, X(:,1),':',LogData.PLRN.TimeS, X(:,2),':');
hold on; set(gca,'ColorOrderIndex',1);

plot(LogData.PLRN.TimeS, polarTrue.Cd0*ones(size(LogData.PLRN.TimeS)),'--',LogData.PLRN.TimeS, polarTrue.B*ones(size(LogData.PLRN.TimeS)),'--');
grid on; grid minor; 
xlabel('Time [s]'); ylabel('Coefficient');
legend('CD0','B');

figure,plot(LogData.PLRN.TimeS, P(:,[1,2,4]));
legend({'P_{Cd0}','{P_cross}','P_B'})

figure,plot(LogData.TECS.TimeS, LogData.TECS.spdem,'b-',LogData.PLRN.TimeS, LogData.PLRN.aspd,'r.')
hold on,plot(LogData.TECS.TimeS, LogData.TECS.sp,'k-')
grid on

% Evaluate final variation.
vz = zeros(size(LogData.PLRN.TimeS));
for iT=1:length(LogData.PLRN.TimeS)
    vz(iT) = -GetSinkSpeed(LogData.PLRN.aspd(iT), polarFinal);
end
deviation = vz - LogData.PLRN.z;

fprintf('Mean %3.2f std %3.2f\n', mean(deviation), std(deviation));
