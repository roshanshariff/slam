#------ start of myscript -------------
#!/bin/sh

#
# When executing, run script in directory where qsub was initially
# called.  This is needed by MATLAB.  Regular scripts should
# work fine.
# $PBS_O_WORKDIR is a variable set by the qsub command and is
# equal to the path of the directory where qsub is called (ie.
# the working directory)

#cd $PBS_O_WORKDIR
cd /usr/eureka/szepesva/MCMC-SLAM

# Run matlab command and direct console output to logfile.txt
# Change name logfile.txt for subsequent runs.

/opt/matlab.R2008a/bin/matlab -nodisplay -r Run51 -logfile logfile-5-1.txt

# When done, tell caller that all is well!

exit 0
#------end of myscript -----------------
