function TestPolarEKF_averaged(iSeed)
    % Estimate glide polar params

    if nargin<1
        iSeed=1;
    end
        
    close all;
    
    seeds = [123,456,789];
    rng(seeds(iSeed));
    
    Sim.dT = 0.1;

    Sim.EndTime=2000;
    
    Filter.Type = '2 State';
    
    Aircraft.Cd0 = 0.05;
    Aircraft.B   = 0.045;
    Aircraft.m = 2.0;
    Aircraft.S = 0.45;
%     Aircraft.m = 13/2.205;
%     Aircraft.S = 10.57*0.3048^2;
    Aircraft.k   = 2*Aircraft.m*9.81/(1.225*Aircraft.S); % 2*W/rho*S
    
    Sim.Time = (0:Sim.dT:Sim.EndTime)';
    Sim.nT = numel(Sim.Time);
    
    % Set up state histories
    Sim.V_std  = 0.44;
    Sim.Vz_std = 0.3;
    
    
    if (0)
        Sim.VTarget = SetupScenario(Sim.Time);
        if (0)
            % Second oder filter representation.
            Sim.States.V = Sim.VTarget(1)*ones(Sim.nT,1);

            inertia=1;
            vdot = 0;
            for i = 2:Sim.nT
                acc = (randn*10*Sim.V_std+(1.0*(Sim.VTarget(i)-Sim.States.V(i-1))-1.0*vdot)/inertia);
                vdot = vdot + Sim.dT*acc;
                Sim.States.V(i)= Sim.States.V(i-1) + vdot*Sim.dT;
            end
        else
            % Typical airspeed variation, from log.
            Variation = load('AspdPCVariation');
            Idx = 1:Sim.nT;
            Sim.States.V = Sim.VTarget .* (1 + Variation.var(1+mod(Idx-1,length(Variation.var)))');
        end
        
        Sim.States.Vz = GetSinkSpeedNonQuadratic(Sim.States.V,Aircraft) + randn(Sim.nT,1)*Sim.Vz_std;

        % It can only change so quickly.
        for iT = 2:Sim.nT
            a = 0.2;
            Sim.States.Vz(iT) = Sim.States.Vz(iT-1)*(1-a) + Sim.States.Vz(iT)*a;
        end
        Sim.States.h = 100 - Sim.dT*cumtrapz(Sim.States.Vz);
    
        % Work out what the measurements and inputs were.
        Measurement.V_std = 0.1;
        Measurement.h_std = 0.1;

        Filter.Measurement.h = Sim.States.h + Measurement.h_std*randn(Sim.nT,1);
        Filter.Measurement.V = Sim.States.V + Measurement.V_std*randn(Sim.nT,1);
        Filter.Measurement.VSpeed = Sim.States.Vz;
        
        Filter.Measurement.Throttle = zeros(Sim.nT,1);
        
        Filter.Inputs.Roll = zeros(Sim.nT,1);
        Filter.Measurement.Time = Sim.Time;
        
        Sim.Roll     = zeros(Sim.nT,1);
        Sim.Pitch    = zeros(Sim.nT,1);
        Sim.AccX     = zeros(Sim.nT,1);
        Sim.AccY     = zeros(Sim.nT,1);
        Sim.AccZ     = zeros(Sim.nT,1);
        Sim.Throttle = zeros(Sim.nT,1);

    else
        % Load log file
        % Typical airspeed variation, from log.
        addpath('~/ardupilog');
            
%         LogData = Ardupilog('00000003.BIN');
        LogData = Ardupilog('Ex2.BIN');
            
        getField=@(f,x) LogData.(f).(x);
        
        Sim.Time = getField('CTUN','TimeUS')/1e6;
        Sim.nT = length(Sim.Time);
        interpField = @(f,x) interp1(getField(f,'TimeUS')/1e6,getField(f,x),Sim.Time); 
        
        
        Sim.Roll  = getField('CTUN','Roll');
        Sim.Pitch = getField('CTUN','Pitch');

        
        Sim.States.V = interpField('TECS','sp');
        
        Sim.VTarget  = interpField('TECS','spdem');
        Sim.States.h = interpField('TECS','h');
        Sim.Throttle = interpField('CTUN','ThrOut');
        Sim.AccX     = interpField('IMU', 'AccX');
        Sim.AccY     = interpField('IMU', 'AccY');
        Sim.AccZ     = interpField('IMU', 'AccZ');
        Sim.VSpeed   = interpField('TECS', 'dh');

        
        Filter.Measurement.Time = Sim.Time;
        Filter.Measurement.h = Sim.States.h;
        Filter.Measurement.V = Sim.States.V;
        Filter.Measurement.Throttle = Sim.Throttle;
        Filter.Measurement.VSpeed = Sim.VSpeed;
        Filter.Inputs.Roll = Sim.Roll;
    end

    % Set up Polar Filter.
    
    Filter.Rate = 1;
    Filter.dT = Sim.dT*Filter.Rate;
    Filter.h_std = 1/sqrt(Filter.Rate);
    Filter.V_std = 0.2/sqrt(Filter.Rate);
    Filter.Vz_std = 0.5/sqrt(Filter.Rate);
    Filter.MinV = 5;
    
    %Filter.Pinit = [0.005, 0.005, 5,   0.05];
    Filter.Pinit = [0.005, 0.005, 5,   0.5];
    %Filter.Q     = [  0,    0, 0.5, 0.2];
    Filter.Q     = [  0,    0, 0.1, 0.05];
    Filter.R     = [Filter.h_std, Filter.V_std];

    Filter.xinit = [0.020;...
                    0.030;...
                    Sim.States.h(1) + 5    * randn;...
                    Sim.States.V(1) + 0.2  * randn];
                
    Filter.StateLabels = {'Cd0','B','Height','Velocity'};
    Filter.InputLabels = {'Height','Velocity'};

    Filter.AircraftK = Aircraft.k;
        
    switch Filter.Type
    case '4 State'
        % Normal filter
    case '5 State'
        % Like normal but also estimates k=2*W/rho*S. This does not work
        % very well because it is possible to have the wrong parameter
        % values produce very nearly the right polar.
        Filter.Pinit = [Filter.Pinit,1];
        Filter.Q     = [Filter.Q,1e-2];
        
        Filter.xinit = [Filter.xinit; Aircraft.k+1*randn];
        
        Filter.StateLabels = [Filter.StateLabels, 'K'];
    case '2 State'
        % Doesn't estimate V and h, just uses them directly.
        Filter.Pinit = Filter.Pinit(1:2);
        Filter.Q = Filter.Q(1:2);
        Filter.R     = Filter.Vz_std;
        Filter.xinit = Filter.xinit(1:2);
        Filter.StateLabels = Filter.StateLabels(1:2);
    end



    Filter.Inputs.h = zeros(Sim.nT,1);
    Filter.Inputs.V = Sim.VTarget;

    % Run actual filter iteratively.
    Filter = RunPolarFilter(Filter);
    
    disp('Done filter run, start plotting');
    
    figure('Position',[680 62 1220 1036]);
        
    subplot(2,2,1);
    hold on;
    activeTime = Sim.Time(~Filter.Active);
    plot([activeTime,activeTime]',[0*activeTime,25+0*activeTime]','-','Color',[1,0.5,0.5]);
    
    h=plot(Sim.Time,Filter.Measurement.V,...
           Sim.Time,Filter.States(:,2),...
           Sim.Time,Sim.States.V,...
           Sim.Time,Sim.VTarget);
     
    
    title('Velocity');
    xlabel('Time [s]'); ylabel('Vel [m/s]');
    legend(h,'Meas','Estimate','Actual','Target');
    grid on; grid minor;
    
    subplot(2,2,2);
    plot(Sim.Time,Filter.Measurement.h,...
         Sim.Time,Sim.States.h);
    title('Height');
    leg = {'Measured','Actual'};
    
    switch Filter.Type
        case {'4 State','5 State'}
            hold on;
            plot(Sim.Time,Filter.States(:,3));
            leg = [leg,'Estimate'];
    end
    xlabel('Time [s]'); ylabel('Height [m]');
    legend(leg);
    grid on; grid minor;
    


    subplot(2,2,3);
    plot(Sim.Time,Aircraft.k./cosd(Sim.Roll)./(Sim.States.V.^2));
    title('Lift coef');
%     set(gca,'YLim',[0,1]);
    grid on; grid minor;
    xlabel('Time [s]'); ylabel('C_L []');
    
    subplot(2,2,4);
    hold on;
    plot(Sim.Time,Sim.Roll);
    plot(Sim.Time,Sim.Pitch);
    plot(Sim.Time,Sim.Throttle);
    plot(Sim.Time,Sim.AccX);
    plot(Sim.Time,Sim.AccY);
    plot(Sim.Time,Sim.AccZ);

    grid on; grid minor;
    
    legend('Roll','Pitch','Throttle','AccX','AccY','AccZ');
    xlabel('Time [s]'); ylabel('Value');
    
    Vx = 5:0.1:20;

    VzInit   = GetSinkSpeed(Vx, Filter.InitialEstimate);
    VzFinal  = GetSinkSpeed(Vx, Filter.FinalEstimate);
    VzActual = GetSinkSpeedNonQuadratic(Vx,Aircraft);
    
    % Plot the filter active periods too.
    VVel.Time = Sim.Time(Filter.Active);
    VVel.h    = Sim.States.h(Filter.Active);
    VVel.Vx   = Sim.States.V(Filter.Active);
    
    VVelD.Vz   = -diff(VVel.h)./diff(VVel.Time);
    VVelD.Vx   = (VVel.Vx(1:(end-1)) + VVel.Vx(2:end))/2;
   
    % Manaul extract
    Period(1).T = [176,241.6];
    Period(1).H  = [323,295];
    Period(1).Vx = 10.65;
    
    Period(2).T = [176,241.6];
    Period(2).H  = [323,295];
    Period(2).Vx = 10.65;
    
    Period(3).T = [176,241.6];
    Period(3).H  = [323,295];
    Period(3).Vx = 10.65;
    
    Period(4).T = [176,241.6];
    Period(4).H  = [323,295];
    Period(4).Vx = 10.65;
    
    
    
    figure('Position',[10 62 1220 1036]);hold on;
    subplot(2,2,1);
    plot(VVelD.Vx,VVelD.Vz,'r.',...
        Vx,VzInit,'b',...
        Vx,VzFinal,'g',...
        Vx,VzActual,'k');
    legend('Measured','Init est.','Final est.','Actual');
    xlabel('Vx [m/s]'); ylabel('Vz [m/s]');
    grid on; grid minor;
    
    subplot(2,2,2); hold on;
    title('State convergence');

    plot(Sim.Time,Filter.States(:,1:2),'-');
    
    set(gca,'ColorOrderIndex',1);
    plot(Sim.Time,[Aircraft.Cd0*ones(Sim.nT,1),Aircraft.B*ones(Sim.nT,1)],'--');
    legend(Filter.StateLabels(1:2));
    
    if strcmp(Filter.Type,'5 State')
        figure,hold on;
        plot(Sim.Time,Filter.States(:,5),'-');
        title('2*W/(rho*S)');
        plot(Sim.Time,Aircraft.k*ones(Sim.nT,1),'--');
    end
    xlabel('Time [s]');
    grid on; grid minor;
    
    subplot(2,2,3);hold on;
    
    ParameterP = Filter.P(:,1:2,1:2);
    ParameterL = Filter.StateLabels(1:2);
    
    plot(Sim.Time,reshape(ParameterP,Sim.nT,[]));
    for i=1:length(ParameterL)
        for j=1:length(ParameterL)
            pLabels{i,j} = ['P_',ParameterL{i},'-',ParameterL{j}];
        end
    end
    legend(pLabels(:),'Interpreter','none');
    title('Parameter variances');
    xlabel('Time [s]');
    
    grid on; grid minor;
    
    subplot(2,2,4);
    plot(Sim.Time,Filter.Resid,'.-');
    title('Residuals');
    legend(Filter.InputLabels);
    grid on; grid minor;
    xlabel('Time [s]');
end

function FlightSpeed = SetupScenario(Time)
    
    FlightSpeed = 8*ones(size(Time));
    
    FlightSpeed(Time>max(Time)*1/8) = 12;
    FlightSpeed(Time>max(Time)*2/8) = 7;
    FlightSpeed(Time>max(Time)*3/8) = 12;
    
    FlightSpeed(Time>max(Time)*4/8) = 7;
    FlightSpeed(Time>max(Time)*5/8) = 18;
    FlightSpeed(Time>max(Time)*6/8) = 7;
    FlightSpeed(Time>max(Time)*7/8) = 20;

    
    % Apply ramp rate limitation
    V = FlightSpeed;
    maxDelta = 1*mean(diff(Time));
    for i=2:length(Time)
        delta = V(i)-V(i-1);
        V(i) = V(i-1) + sign(delta)*min(maxDelta,abs(delta));
    end
    
    FlightSpeed = V;
end

function Vz=GetSinkSpeed(V, Polar)
    Cl = Polar.k./V.^2;
    Cd =Polar.Cd0 + Polar.B.*Cl.^2;
    Vz = V.*Cd./Cl;
end

function Vz=GetSinkSpeedNonQuadratic(V, Polar)
    Cl = Polar.k./V.^2;
    D = 0.0*Polar.Cd0;
    Cd = Polar.Cd0 + Polar.B.*Cl.^2 + D.*Cl;
    Vz = V.*Cd./Cl;
end

function Filter = RunPolarFilter(Filter)
    % Create a PolarEstimator object and use it to recursively estimate the
    % aircraft drag polar.
    
    polarEstimator = PolarEstimator(Filter.MinV,...
                                    Filter.Type,...
                                    Filter.Pinit,...
                                    Filter.xinit,...
                                    Filter.Q,...
                                    Filter.R,...
                                    Filter.dT,...
                                    Filter.AircraftK);
    
    for i=1:size(Filter.Measurement.V,1)
        % Recursive update
        polarEstimator.update(Filter.Measurement.Time(i),...
                              Filter.Measurement.V(i),...
                              Filter.Measurement.h(i),...
                              Filter.Measurement.Throttle(i),...
                              Filter.Inputs.V(i),...
                              Filter.Inputs.Roll(i),...
                              Filter.Measurement.VSpeed(i));
                          
        % State logging
        Filter.States(i,:) = polarEstimator.ekf.x;
        Filter.Resid(i,:)  = polarEstimator.ekf.residual';
        Filter.P(i,:,:)    = polarEstimator.ekf.P;
        Filter.Active(i)   = polarEstimator.Active;
    end

    Filter.InitialEstimate.Cd0 = Filter.xinit(1);
    Filter.InitialEstimate.B   = Filter.xinit(2);
    
    Filter.FinalEstimate.Cd0 = Filter.States(end,1);
    Filter.FinalEstimate.B   = Filter.States(end,2);
    
    if strcmp(Filter.Type, '5 State')
        % K estimated
        Filter.InitialEstimate.k = Filter.xinit(5);
        Filter.FinalEstimate.k   = Filter.States(end,5);
    else
        % K provided
        Filter.InitialEstimate.k = Filter.AircraftK;
        Filter.FinalEstimate.k   = Filter.AircraftK;
    end
end
