function h = PlotLogs(logs, category, fields, lineTypes, names, ylabels)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here

    if ~iscell(fields)
        fields = {fields};
    end
    if ~iscell(category)
        category = repmat({category},length(fields),1);
    end
    
    figure;
    colors = get(gca,'ColorOrder');
    
    if nargin<4 || isempty(lineTypes)
        lineTypes = {'-',':','-.','--'};
    end
    
    if nargin<5
        names = arrayfun(@(x) ['Log ',num2str(x)],1:length(logs),'UniformOutput',false);
    end
    
    if nargin<6
        ylabels = [category{1},'.',fields{1}];
    end
    
    hold on;
    h=[];
    for iL=1:length(logs)
        for iF=1:length(fields)
            mult=1;
            field=fields{iF};
            if field(1)=='@'
                field=field(2:end);
                if field(1)=='-'
                    mult=-1;
                    field=field(2:end);
                end
            end     
            iC=mod(iL-1,length(colors))+1;
            h(end+1)=plot(logs(iL).(category{iF}).Time,mult*logs(iL).(category{iF}).(field),'Color',colors(iC,:),'LineStyle',lineTypes{iF});
        end
    end
    
    legend(names,'Interpreter','none');

    xlabel('Time [s]'); ylabel(ylabels);
    grid on; grid minor;
end

