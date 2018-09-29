function logs=AddSoaringData(logs, thermal)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    for i=1:length(logs)
        logs(i).SOAR.posN = interp1(logs(i).NKF1.TimeUS, logs(i).NKF1.PN, logs(i).SOAR.TimeUS);
        logs(i).SOAR.posE = interp1(logs(i).NKF1.TimeUS, logs(i).NKF1.PE, logs(i).SOAR.TimeUS);
       

        logs(i).SOAR.estPosN = logs(i).SOAR.posN + logs(i).SOAR.x2;
        logs(i).SOAR.estPosE = logs(i).SOAR.posE + logs(i).SOAR.x3;
        
        d = sqrt((logs(i).NKF1.PE - thermal.pos(2)).^2+(logs(i).NKF1.PN - thermal.pos(1)).^2);
        logs(i).SOAR.dist = interp1(logs(i).NKF1.Time,d,logs(i).SOAR.Time);
        
        logs(i).SOAR.posErr = sqrt(sum(([logs(i).SOAR.estPosN,logs(i).SOAR.estPosE]  -thermal.pos).^2 ,2));
    end 
end
