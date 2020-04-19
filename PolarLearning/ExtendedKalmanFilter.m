classdef ExtendedKalmanFilter < handle
    %ExtendedKalmanFilter Filter to estimate non-linear system state from
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
        k = 30;
        residual=[0;0];
        jacobian_f;
        jacobian_h;
        inputUpdate;
    end
    
    methods
        function obj=ExtendedKalmanFilter(Pinit,xinit,Q,R,jacobian_f,jacobian_h)
            obj.P=Pinit;
            obj.x=xinit;

            obj.Q=Q;
            obj.R=R;
            
            obj.jacobian_f = jacobian_f;
            obj.jacobian_h = jacobian_h;
        end
        function update(ekf,z,u)
            %Estimate new state from old. Also obtain Jacobian matrix for
            %later.
            [x2,A]=ekf.jacobian_f(ekf.x, u);
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








