function logs = NormaliseTimes(logs)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    for i=1:length(logs)
        
        if length(logs(i).NKF1.TimeUS) > 1
            startTimeUS = logs(i).NKF1.TimeUS(1);
        else
            startTimeUS = logs(i).ATT.TimeUS(1);
        end
        
        fields = fieldnames(logs(i));
        
        for iF=1:length(fields)
            field=fields{iF};
            if isfield(logs(i).(field),'TimeUS')
                logs(i).(field).TimeUS = double(logs(i).(field).TimeUS - startTimeUS);
                logs(i).(field).Time   = logs(i).(field).TimeUS/1e6;
            end
            if isfield(logs(i).(field),'TimeS')
                logs(i).(field).TimeS = double(logs(i).(field).TimeS - double(startTimeUS)/1e6);
            end
        end
    end 
end

