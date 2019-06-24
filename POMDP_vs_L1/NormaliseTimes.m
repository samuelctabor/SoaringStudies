function logs = NormaliseTimes(logs)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    for i=1:length(logs)
        
        startTime = logs(i).NKF1.TimeUS(1);
        
        fields = fieldnames(logs(i));
        
        for iF=1:length(fields)
            field=fields{iF};
            if isfield(logs(i).(field),'TimeUS')
                logs(i).(field).TimeUS = double(logs(i).(field).TimeUS - startTime);
                logs(i).(field).Time = logs(i).(field).TimeUS/1e6;
            end
            
            if isfield(logs(i).(field),'TimeS')
                logs(i).(field).TimeS = double(logs(i).(field).TimeS - startTime/1e6);
                logs(i).(field).Time = logs(i).(field).TimeS;
            end
        end
    end 
end

