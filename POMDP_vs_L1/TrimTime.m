function Log=TrimTime(Log,ends, resetToZeroStart)

    if nargin<3
        resetToZeroStart = false;
    end
    
    fields=fieldnames(Log);

    for iF=1:length(fields)
        if isfield(Log.(fields{iF}),'TimeUS')
            idx = Log.(fields{iF}).Time>ends(1) & Log.(fields{iF}).Time<ends(2);
            fields2=fieldnames(Log.(fields{iF}));
            sz = size(Log.(fields{iF}).TimeS);
            for iF2=1:length(fields2)
                if all(size(Log.(fields{iF}).(fields2{iF2}))==sz)
                    Log.(fields{iF}).(fields2{iF2}) = Log.(fields{iF}).(fields2{iF2})(idx);
                end
            end
            
            if resetToZeroStart
                Log.(fields{iF}).Time = Log.(fields{iF}).Time - ends(1);
            end
        end
    end
end