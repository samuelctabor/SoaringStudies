function [f1,a,b] = fitCurve(t,x, nbases)

    % Interpolate onto 2^N.
    n = 2^nextpow2(length(t));
    
    ti = linspace(min(t),max(t),n);
    xi = interp1(t,x,ti);
    
   [f,P,ang] = singleSidedFFT(ti,xi(1:end-1));
  
   % cos to sin
   ang = ang + pi/2;
   
%    f1 = f(1:nbases);
%    a = P(1:nbases);
%    b = ang(1:nbases);
   f1 = f(2:nbases+1);
   a = P(2:nbases+1);
   b = ang(2:nbases+1);
   
   if (0)
       figure,subplot(2,1,1);
       plot(f,P);
       subplot(2,1,2);
       plot(f,ang);
       
       figure,plot(t,x,'b-x',ti,xi,'b-o',ti, P*sin(2*pi*f'*ti + ang'),'r-o');

       hold on;
       plot(ti, P(1) + a*sin(2*pi*f1'*ti + b'),'g-o');
       plot(t,  P(1) + a*sin(2*pi*f1'*t + b'),'g-x');
       
       t2 = linspace(min(t),max(t),1000);
       x2 = P(1) + a*sin(2*pi*f1'*t2 + b');
       plot(t2,x2 ,'k-');
   end
   
  f1=f1(:);
   a = a(:);
   b = b(:);
end

function [f, P1, ang1]=singleSidedFFT(t,S)
    L = length(t);
    % Equivalent to the first method below as mean(diff(t))=(max(t)-min(t))/L
%     Fs = 1/mean(diff(t));
%     f = Fs*(0:(L/2))/L;

    % If this line is used, we get a good match to the input data but there
    % is high-frequency content that shows up in the velocities.
    f = (0:L/2)/(max(t)+min(t)/L);

    % If this line is used, the input data is not matched but we don't get
    % so much high frequency content.
%     f = (0:L/2)/max(t);

    Y = fft(S);
    P2 = abs(Y/L);
    ang2 = angle(Y);

    P1 = P2(1:L/2+1);
    ang1 = ang2(1:L/2+1);

    P1(2:end-1) = 2*P1(2:end-1);
end