classdef PolarEstimator < handle
    % PolarEstimator Estimates aircraft drag polar from noisy measurements
    % using EKF.

    properties (SetAccess=protected)
        ekf;
        MinV = 5;
        PreviousTime = 0;
        Reset = false;
        MaxDeltaT = 0.2;
        MinGlideTime = 10;
        GlideTime = 0;
        PreviousVTarget = 0;
        PreviousThrottle = 0;
        Active=false;
        dT = 0;
        k = 0;
        roll = 0;
    end
    
    methods
        function obj=PolarEstimator(MinV, Type, Pinit, xinit, Q, R, dT, Kaero)
            obj.MinV = MinV;
            obj.dT = dT;
            obj.k = Kaero;
            
            if (0)
                % Use autogenerated functions
                consts = [obj.k,obj.dT];
                F_func = @(x,y) deal(ff_autogen(x,y,consts), F_autogen(x,y,consts));
                H_func = @(x,y) deal(hh_autogen(x,y,consts), H_autogen(x,y,consts));
            else 
                F_func = @(x,u) obj.jacobian_f_4state(x,u);
                H_func = @(x,u) obj.jacobian_h(x,u);
            end
            
            switch Type
                case '4 State'
                    obj.ekf = ...
                        ExtendedKalmanFilter( diag(Pinit.^2),...
                                            xinit,...
                                            diag(Q.^2),...
                                            diag(R.^2),...
                                            F_func,...
                                            H_func);

                case '5 State'
                    obj.ekf = ...
                        ExtendedKalmanFilter( diag(Pinit.^2),...
                                            xinit,...
                                            diag(Q.^2),...
                                            diag(R.^2),...
                                            @obj.jacobian_f_5state,...
                                            @obj.jacobian_h,...
                                            @obj.inputUpdate);
                otherwise
                    error('Unrecognised type!');
            end
        end
        
        function update(obj, Time, Speed, Height, Throttle, VTarget,Roll)
            obj.roll = Roll;
            obj.Active=false;
            if Throttle==0 && obj.PreviousThrottle>0
                obj.GlideTime = Time;
            end
            
            if  Throttle>0 || ...
                Speed<obj.MinV || ...
                any(isnan([Speed,Height,Throttle,VTarget,Roll])) || ...
                abs(Roll)>5 || ...
                abs(VTarget-obj.PreviousVTarget)/(Time-obj.PreviousTime) > 0.1
                    
                % Conditions not suitable. Filter will need to be reset.
                obj.GlideTime = Time;
                obj.Reset = true;
            elseif (Time - obj.GlideTime)<obj.MinGlideTime
                obj.Reset = true;
            elseif obj.Reset
                % Conditions now suitable, but reset required.
                obj.Reset = false;
                
                obj.ekf.reset_state([Height;...
                                    Speed;...
                                    obj.ekf.x(3);...
                                    obj.ekf.x(4)]);
            else
                obj.Active = true;
                % Conditions suitable and no reset required.
                
                FilterMeasurement = [Height;Speed];
                FiltInputs = [0; VTarget - obj.PreviousVTarget; 0;0];

                obj.ekf.update(FilterMeasurement,FiltInputs);
            end
            
            obj.PreviousVTarget = VTarget;
            obj.PreviousTime = Time;
            obj.PreviousThrottle = Throttle;
        end
       
        function [w,A]=jacobian_h(~,x,~)
            % Measurements
            A=zeros(2,numel(x));
            A(1,1)=1;
            A(2,2)=1;
            w = A*x;
        end
        
        function [xn,A]=jacobian_f_4state(obj,x,u)
             %Computes new state and jacobian
            % States:
            % [h;V;Cb0;B]
            % New state
            xn=x + u;
            % h' = h - dT*Vz;
            % h' = h - dT*(Cd0*Vz^3/k + B*k/Vz);
            Cl = obj.k/cosd(obj.roll)/(x(2)^2);
            
            Cd = x(3) + x(4)*Cl^2;
            
            xn(1) = x(1) - obj.dT * x(2) * Cd/Cl;
%             xn(1) = x(1) - obj.dT*(x(3)*x(2)^3/obj.k + x(4)*obj.k/x(2));
            
            % Partial deivates of xn w.r.t x
            A = eye(4,4);
            
            A(1,2) = -3*x(3)*obj.dT*x(2)^2/obj.k + x(4)*obj.k*obj.dT/x(2)^2;
            A(1,3) = -x(2)^3*obj.dT/obj.k;
            A(1,4) = -obj.k*obj.dT/x(2);
        end
        
        function [xn,A]=jacobian_f_5state(x,dT)
            %Computes new state and jacobian
            % States:
            % [h;V;Cb0;B;K]
            % New state
            xn=x;
            % h' = h - dT*Vz;
            % h' = h - dT*(Cd0*Vz^3/k + B*k/Vz);
            xn(1) = x(1) - dT*(x(3)*x(2)^3/x(5) + x(4)*x(5)/x(2));
            
            % Partial deivates of xn w.r.t x
            A = eye(5,5);
            
            A(1,2) = -3*x(3)*dT*x(2)^2/x(5) + x(4)*x(5)*dT/x(2)^2;
            A(1,3) = -x(2)^3*dT/x(5);
            A(1,4) = -x(5)*dT/x(2);
            A(1,5) = dT*(x(3)*x(2)^3/x(5)^2 + x(4)/x(2));
        end
        
        function xn = inputUpdate(~,x)
            xn = x;
        end
    end
end








