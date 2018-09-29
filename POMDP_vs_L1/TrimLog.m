function logs=TrimLog(logs,endTimes)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    for i=1:length(logs)
        fields = fieldnames(logs(i));
        
        for iF=1:length(fields)
            field=fields{iF};
            if isfield(logs(i).(field),'TimeUS')
                idx = logs(i).(field).TimeUS/1e6 > endTimes(1) & ...
                      logs(i).(field).TimeUS/1e6 < endTimes(2);
                  
                
                
                fields2=fieldnames(logs(i).(field));
                
                for iF2=1:length(fields2)
                    field2=fields2{iF2};
                    
                    logs(i).(field).(field2) = logs(i).(field).(field2)(idx);
                end
            end
        end
    end 
end

