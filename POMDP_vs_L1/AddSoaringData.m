function logs=AddSoaringData(logs, thermal)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    if nargin<2
        thermal.pos = [0,0];
    end

    for i=1:length(logs)
        
        logs(i).SOAR.posN = logs(i).SOAR.north;
        logs(i).SOAR.posE = logs(i).SOAR.east;

        logs(i).SOAR.estPosN = logs(i).SOAR.x2;
        logs(i).SOAR.estPosE = logs(i).SOAR.x3;
        
        logs(i).SOAR.dist = sqrt((logs(i).SOAR.east - thermal.pos(2)).^2+(logs(i).SOAR.north - thermal.pos(1)).^2);
        
        logs(i).SOAR.posErr = sqrt(sum(([logs(i).SOAR.estPosN,logs(i).SOAR.estPosE]  -thermal.pos).^2 ,2));
    end 
end
