classdef ExtendedKalmanFilter_polar2state < handle
    %ExtendedKalmanFilter_polar2state Filter to estimate non-linear system state from
    %noisy measurements.
    %   Class maintains state estimation, covariance of measurements and
    %   state covariance estimation. Update method takes sensor readings
    %   and performs the necessary calcs. Note it also takes a function
    %   handle to the state update function and measurement estimation
    %   function.

    properties (SetAccess=protected)
        P=zeros(4,4);
        x=zeros(4,1);
        Q=zeros(4,4);
        R=zeros(1,1);
        dT = 0.01;
        residual=[0;0];
    end
    
    methods
        function obj=ExtendedKalmanFilter_polar2state(Pinit,xinit,Q,R)
            obj.P=Pinit;
            obj.x=xinit;

            obj.Q=Q;
            obj.R=R;
            
        end
        function update(ekf,z,u)
            %Estimate new state from old. Also obtain Jacobian matrix for
            %later.
            
            x1 = ekf.x;
            x1(1:2) = max(x1(1:2),[0.02;0.02]);
            
            [x2,A]=ekf.jacobian_f(x1, u);
            %Add inputs effects
            
            %What measurement do we expect to receive in the estimated
            %state
            [z1,H] = ekf.jacobian_h(x2, u);    
            
            %Update the covariance matrix 
            Pn = A*ekf.P*A'+ekf.Q;                 %partial update

            %Calculate the KALMAN GAIN
            P12 = Pn*H';                               %cross covariance
            K = P12/(H*P12+ekf.R);                     %Kalman filter gain
            
            %Correct the state estimate using the measurement residual.
            ekf.x = x2+K*(z-z1);
            
            ekf.residual = (z-z1);
            
            %Correct the covariance too.
            ekf.P = Pn-K*P12';
            
            % Ensure P matrix is symmetric
            ekf.P = ekf.make_symmetric(ekf.P);                      
        end
        function reset(ekf,x,P)
            %Reset covariance and state.
            ekf.x=x;
            ekf.P=P;
        end
        function reset_state(ekf,newState)
            ekf.x = newState;
        end
        
        function [w,A]=jacobian_h(~,x,u)
             % Pred measurement
            V = u(1);
            roll = u(2);
            k = u(3);
            
            A=zeros(1,numel(x));
            
            A(1) = cosd(roll)*V^3/k;
            A(2) = k / (cosd(roll)*V);
            w = A*x;
        end
        
        function [xn,A]=jacobian_f(~,x,~)
            %Computes new state and jacobian
            % States:
            % [Cb0;B]
            
            % New state
            xn=x;
            
            % Partial deivates of xn w.r.t x
            A = eye(2,2);
        end
        
    end
    methods(Static)
        function B=make_symmetric(A)
            B=A;
            for i=2:size(A,1)
                for j=1:i-1
                    B(i,j)=(B(i,j)+B(j,i))/2;
                    B(j,i)=B(i,j);
                end
            end
        end            
    end
end








