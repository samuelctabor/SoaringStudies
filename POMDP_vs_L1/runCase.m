function Log=runCase(name, params)
% Run a specific set of parameters and return a log file.
    
%     ardupilotDir = '/home/samuel/ArdupilotDev/ardupilot/ArduPlane';
    ardupilotDir = '/home/samuel/Personal/ardupilot/ArduPlane';

    % Call the STIL
    speedUp=1;
    
    cmd = sprintf('sim_vehicle.py -f plane-soaring --speedup %i', speedUp);
    
    runInTerminalWindow=true;
    if runInTerminalWindow
        system(['cd ', ardupilotDir, ' && LD_LIBRARY_PATH= && gnome-terminal -e "', cmd, '" &'])
    else
        system(['cd ', ardupilotDir, ' && LD_LIBRARY_PATH= && ', cmd, ' &']);
    end
    
    % Call the python library that manages the parameters and monitors the
    % sim. Block until done.
    
    args='';
    
    if nargin>1
        paramNames = fieldnames(params);
        for i=1:length(paramNames)
            args = [args, sprintf('%s=%f ', paramNames{i}, params.(paramNames{i}))];
        end
    end
    
    cmd = ['LD_LIBRARY_PATH= && python runSoaring.py ', args];
    system(cmd);
    
    system('killall -9 python');
    
    % Convert and read the resulting log file.
    fprintf('Reading log . . .');
    fid = fopen([ardupilotDir,'/logs/LASTLOG.TXT']);
    log_ID = str2double(fgetl(fid));
    fclose(fid);
    
    logName = sprintf('%s/logs/%08i.BIN',ardupilotDir,log_ID);
    
    delete([name,'.mat']);
    log = Ardupilog(logName);
    Log = log.getStruct();
    save([name,'.mat'],'-struct','Log');
end

