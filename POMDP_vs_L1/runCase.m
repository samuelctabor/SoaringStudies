function Log=runCase(name, params)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    
    ardupilotDir = '/home/samuel/ArdupilotDev/ardupilot/ArduPlane';

    % Call the STIL
    speedUp=1;
    
%     cmd = sprintf('sim_vehicle.py -f plane --speedup %i -GDB "AP_L1_Control.cpp:437"', speedUp);
%     cmd = sprintf('sim_vehicle.py -f plane --speedup %i -GDB "Attitude.cpp:573"', speedUp);
    cmd = sprintf('sim_vehicle.py -f plane --aircraft POMDP_vs_L1 -w --add-param-file="../Tools/autotest/default_params/plane-soaring.parm" --speedup %i', speedUp);
%     cmd = '/home/samuel/ArdupilotDev/ardupilot/build/sitl/bin/arduplane -S -I0 --home -35.363261,149.165230,584,353 --model plane --speedup 20 --defaults /home/samuel/ArdupilotDev/ardupilot/Tools/autotest/default_params/plane.parm';
    
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

