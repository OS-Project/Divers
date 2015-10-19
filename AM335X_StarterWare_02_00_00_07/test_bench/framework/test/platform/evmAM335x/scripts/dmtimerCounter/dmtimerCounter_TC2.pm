#
# Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/ 
#
# 
#  Redistribution and use in source and binary forms, with or without 
#  modification, are permitted provided that the following conditions 
#  are met:
#
#    Redistributions of source code must retain the above copyright 
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the 
#    documentation and/or other materials provided with the   
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#
sub dmtimerCounter_TC2($)
{
    my $indexTC = $_[0];
	my $appLog = 0;
	my $TempTeraTermMacroFile = $TeraTermMacroFile."_temp.ttl";
	my $inFileName = $tableTestCases[$locLoopCount]{"testCaseId"}{"scriptName"};
	
	$inFileName =~ s/\.pm/\.txt/;
	
	copy($TeraTermMacroFile,$TempTeraTermMacroFile) or die "Copy failed: $!";
	
	open (hTempTTLFile, ">>".$TempTeraTermMacroFile) || die "***".$TempTeraTermMacroFile." file open failed ***\n";
	open (hTTLSendFileTemplate, $rootpath."/".$configFilePath."SendData_Template_TTL.txt") || die "***".$rootpath."/".$configFilePath."SendData_Template_TTL.txt"." file open failed ***\n";
	until(eof(hTempTTLFile))
	{
		my $readLine = <hTempTTLFile>;
	}

	print hTempTTLFile ("\n");
	until(eof(hTTLSendFileTemplate))
	{
		my $readLine = <hTTLSendFileTemplate>;
		
		if($readLine =~ /<SCRIPT_FILES_PATH><SCRIPT_FILE_NAME>/)
		{
			$readLine =~ s/<SCRIPT_FILES_PATH><SCRIPT_FILE_NAME>/$rootpath\/$scriptsFilePath$inFileName/;
		}
		
		print hTempTTLFile ($readLine);
	}

	close(hTempTTLFile);
	close(hTTLSendFileTemplate);
	
	$cmd = "ttermpro.exe /C=1 /F=".$TeraTermConfigFile." /M=".$TempTeraTermMacroFile." /L=".$logFileNameTemp."\n";

	eval
	{
		# wait for the timeout period specified for each application before timing out.
		alarm($tableTestCases[$locLoopCount]{"testCaseId"}{"timeOut"}+100);
		print "Loading BootLoader\n\n\n";

		$pid = system 1, $cmd;
		sleep($tableTestCases[$locLoopCount]{"testCaseId"}{"timeOut"});
		alarm(0);
		
	};	
	
	# timed out
#	if ($@ =~ /TIMED OUT/ )
#	{
#		system("taskkill /im ttpmacro.exe");
#		print "TIMED OUT!!!!!\n\n\n";
#	}
#	else
#	{
#		print "DID NOT TIMED OUT ;-)\n\n\n";
#	}
	
	print "Test case completed for ".$tableTestCases[$locLoopCount]{"testCaseId"}{"binaryName"}."\n";

	$tableTestCases[$indexTC]{"testCaseId"}{"testStatus"} = "Pass";

	open (hLogFileTemp, $logFileNameTemp) || die "***".$logFileNameTemp." file open failed ***\n";
	open (hLogFile, ">".$logFileName) || die "***".$logFileName." file open failed ***\n";
	
	LOGFILELINE: until(eof(hLogFileTemp))
	{
		my $readLine = <hLogFileTemp>;
		
		if($readLine =~ /Jumping to StarterWare Application/)
		{
			$appLog = 1;
			next LOGFILELINE;
		}
		elsif (0==$appLog)
		{
			next LOGFILELINE;
		}
		
		print hLogFile ($readLine);
		
		if(($readLine =~ /ERROR/i)||($readLine =~ /WARNING/i)||($readLine =~ /FAIL/i))
		{
			$tableTestCases[$indexTC]{"testCaseId"}{"testStatus"} = "Failed";
			$tableTestCases[$indexTC]{"testCaseId"}{"failCondition"} = "Application Error";
		}
	}

	if($tableTestCases[$indexTC]{"testCaseId"}{"testStatus"} ne "Failed")
	{
		$tableTestCases[$indexTC]{"testCaseId"}{"testStatus"} = "Pass";
		$tableTestCases[$indexTC]{"testCaseId"}{"failCondition"} = "Application Passed";
	}
	
	close(hLogFile);
	close(hLogFileTemp);
}

1; # return true