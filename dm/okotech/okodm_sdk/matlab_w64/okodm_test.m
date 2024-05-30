function okodm_test()
    % Simple demo,  shows how to use OKO deformable mirrors from MATLAB
	% Can be used with any mirror model and any DAC unit, those may be given as parameters
	%-----------------------------------------
	% Name of the labrary (files: okodm.dll and okodm.h are used)
	LibName='okodm64';

	% load the library if it is not loaded yet
	if not(libisloaded(LibName))
		loadlibrary(LibName,LibName)
	end

	% Open the device, passing type of the mirror, type of the DAC unit and serial number of the DAC, 
	% unit ID is explicitely specified. Keep the handler returned
	%h=calllib(LibName,'okodm_open','MMDM 37ch,15mm','USB DAC 40ch, 14bit',{'D40V2e15'});
    h=calllib(LibName,'okodm_open','MMDM 96ch, embedded control','Embedded HV DAC',{''}); 
	
	% same for automatic module recognition
	%h=calllib(LibName,'okodm_open','MMDM 37ch,15mm','USB DAC 40ch, 14bit',{''})

	% Install a clean up handler, it will be called upon reaching the end of function, 
	% even if terminated abnormally (e.g. by Ctrl-C)
	cleanupObj=onCleanup(@() cleanUp(LibName,h));
	
	% Check if open was successful
	if(h==0) 
		fprintf('Error opening the mirror\n')
		return
    end 		
	
	% get the number of channnels
	chan_n=calllib(LibName,'okodm_chan_n',h);

	% prepare two arrays with control values: maximum(1) and minimum(-1)
	down=-1*ones(chan_n,1);
	up=ones(chan_n,1);

	fprintf('Press <Ctrl-C> to terminate\n')
	
	% Indefinetely switch between maximum and minimum values
	while true
	    
		% set all channels to 1
		calllib(LibName,'okodm_set',h,libpointer('doublePtr',up),chan_n);
		pause(1)
	
	    % set all channels to -1
		calllib(LibName,'okodm_set',h,libpointer('doublePtr',down),chan_n);
		pause(1)
	end	

end % okodm_test()

function cleanUp(LibName,h)
	% close the mirror device if it is open (i.e. non-zero handler)
	if(h~=0) 
		calllib(LibName,'okodm_close',h)
	end

	% unload the library
	if(libisloaded(LibName))
		unloadlibrary(LibName)
	end	
end % cleanUp
