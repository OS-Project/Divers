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


use warnings;
#use strict;
use Cwd;
use threads;
use threads::shared;
use File::Copy;
use Win32; 
use Win32::Process;

$rootpath = getcwd();
if ( @ARGV > 0 )
{
	$inFilePath = "framework/test/platform/".$ARGV[0]."/";
	$sourceFile = $ARGV[1];
}
else
{
	$inFilePath = "framework/testlink_interface/parser/";
	$sourceFile = "listTestCases.txt";
}
$defaultAnalyzerLoaded = 0;
$binaryFilePath = "framework/test/platform/evmAM335x/binaries/";
$binaryFile = "boot.bin";
$configFilePath = "framework/test/teraterm_config/";
$logFilePath = "framework/test/platform/evmAM335x/logs/";
$scriptsFilePath = "framework/test/platform/evmAM335x/scripts/";

$TCStatusFileHeading = "\ntestCaseId,binaryName,scriptName,timeOut,testStatus,failCondition,Remarks\n";
$TCStatusFileName = "TCResults.txt";

$logFileName = "";
$logFileNameTemp = "";

$TeraTermConfigFile    = $rootpath."/".$configFilePath."StarterWare_UART_BL.INI";
$TeraTermMacroFile     = $rootpath."/".$configFilePath."StarterWare_UART_BL.ttl";
$TeraTermMacroTemplate = $rootpath."/".$configFilePath."StarterWare_UART_BL_TEMPLATE.ttl";

$logFileNameTemplate   = "<LOGFILENAME>";

# Data structure to hold the details of each test case (TC)
@tableTestCases = ("testCaseId", ("binaryName", "", "scriptName", "", "timeOut", "", "testStatus", "", "failCondition", "", "Remarks", "", "execType", "", "scriptPath", "", "logPath", ""));

# In file (either raw file OR from TestLink) from which the TC details are fetched.
#if(1==($#ARGV + 1))
#{
#}

open (hSourceFile, $rootpath."/".$inFilePath.$sourceFile) || die "***".$rootpath."/".$inFilePath.$sourceFile." file open failed ***\n";

open (hTCStatusFile, ">".$rootpath."/".$TCStatusFileName) || die "***".$TCStatusFileName." file open failed ***\n";
print hTCStatusFile ($TCStatusFileHeading);
close(hTCStatusFile);

print "--------------------------------\n";
print "TestCase list file read done.\n";
print "--------------------------------\n";

# Globals
$numBinariesToTest = 0;
$NUM_DETAILS = 2;

my $temp;


# Update details from the infile to the data structure. 
GETDETAILSLOOP: until(eof(hSourceFile))
{
	$locLoopCount = 0;
	$searchLine = <hSourceFile>;
	chomp($searchLine);
	
	$searchLine =~ s/[[:cntrl:]]+//;
	$searchLine =~ s/^\s+//;
	
	@searchLineWords = split(/\s+/, $searchLine);
	$searchLineWordcount = @searchLineWords;
	
	if(($searchLineWordcount < $NUM_DETAILS)||($searchLine eq $TCStatusFileHeading))
	{
		next GETDETAILSLOOP;
	}
	
	$tableTestCases[$numBinariesToTest]{"testCaseId"} = $searchLineWords[0];
	$tableTestCases[$numBinariesToTest]{"testCaseId"}{"binaryName"} = $searchLineWords[1];
	$tableTestCases[$numBinariesToTest]{"testCaseId"}{"timeOut"} = $searchLineWords[2];
	$tableTestCases[$numBinariesToTest]{"testCaseId"}{"scriptName"} = $searchLineWords[3];
	$tableTestCases[$numBinariesToTest]{"testCaseId"}{"execType"} = $searchLineWords[4];
	$tableTestCases[$numBinariesToTest]{"testCaseId"}{"testStatus"} = 0;
	$tableTestCases[$numBinariesToTest]{"testCaseId"}{"failCondition"} = 0;
	$tableTestCases[$numBinariesToTest]{"testCaseId"}{"Remarks"} = 0;
	
	print "testCaseId: ".$tableTestCases[$numBinariesToTest]{"testCaseId"}."\n";
	print "binaryName: ".$tableTestCases[$numBinariesToTest]{"testCaseId"}{"binaryName"}."\n";
	print "timeOut: ".$tableTestCases[$numBinariesToTest]{"testCaseId"}{"timeOut"}."\n";
	print "scriptName: ".$tableTestCases[$numBinariesToTest]{"testCaseId"}{"scriptName"}."\n";

	$tstring = $tableTestCases[$numBinariesToTest]{"testCaseId"}{"binaryName"};
	#$tstring =~ s/\.bin.+$//;#gets left side only of .bin from string
	$tstring =~ s/\.bin//;#gets left side only of .bin from string
	$tableTestCases[$numBinariesToTest]{"testCaseId"}{"logPath"} = $rootpath."/".$logFilePath.$tstring;
	mkdir $tableTestCases[$numBinariesToTest]{"testCaseId"}{"logPath"};

	if(($tableTestCases[$numBinariesToTest]{"testCaseId"}{"scriptName"} !~ /[a-z]/i) && ($tableTestCases[$numBinariesToTest]{"testCaseId"}{"scriptName"} !~ /NA/))
	{
		$tableTestCases[$numBinariesToTest]{"testCaseId"}{"scriptPath"} = $rootpath."/".$scriptsFilePath;
		if($defaultAnalyzerLoaded == 0)
		{
			$temp = $rootpath."/".$scriptsFilePath."analyze_default.pm";
			$defaultAnalyzerLoaded = 1;
			#require eval($rootpath."/".$scriptsFilePath."analyze_default.pm");
			require $rootpath."/".$scriptsFilePath."analyze_default.pm";
		}
	}
	elsif($tableTestCases[$numBinariesToTest]{"testCaseId"}{"scriptName"}=~/\.pm/i)
	{		
		$tableTestCases[$numBinariesToTest]{"testCaseId"}{"scriptPath"} = $rootpath."/".$scriptsFilePath.$tstring;

		$temp = $rootpath."/".$scriptsFilePath.$tstring."/".$tableTestCases[$numBinariesToTest]{"testCaseId"}{"scriptName"};
		$temp =~ s/\.pm//;

		require eval("\"".$temp.".pm\"");
	}
	else
	{
		$tableTestCases[$numBinariesToTest]{"testCaseId"}{"scriptPath"} = $rootpath."/".$scriptsFilePath;
		if($defaultAnalyzerLoaded == 0)
		{
			$temp = $rootpath."/".$scriptsFilePath."analyze_default.pm";
			$defaultAnalyzerLoaded = 1;
			require $rootpath."/".$scriptsFilePath."analyze_default.pm";
		}
	}
		
	$numBinariesToTest++;
}

# indication of timeout
$SIG{ALRM} = sub { die "TIMED OUT!!!!!\n" };

$cmd = "staf 192.168.247.107 usbremotepower off 1";
$temp = system ($cmd);
sleep(3);

# Kill Tera Term if already open.
system("taskkill /im ttermpro.exe");

for($locLoopCount=0;$locLoopCount<$numBinariesToTest;$locLoopCount++)
{
	my $process;
	my $pid = 0;

	$cmd = "staf 192.168.247.107 usbremotepower on 1";
	system ($cmd);
	sleep(6);
	# Update boot-file details
	{
		copy($TeraTermMacroTemplate,"temp.txt") or die "Copy failed: $!";
		
		open (hTTMacroTemplateCopy, $TeraTermMacroTemplate) || die "***".$TeraTermMacroTemplate." file open failed ***\n";
		open (hTTMacroTemplate, ">"."temp.txt") || die "***"."temp.txt"." file open failed ***\n";
		until(eof(hTTMacroTemplateCopy))
		{
			my $readLine = <hTTMacroTemplateCopy>;
			
			if($readLine =~ /<BIN_FILES_PATH>/)
			{
				$readLine =~ s/<BIN_FILES_PATH>/$rootpath\/$binaryFilePath/;
			}			
			if($readLine =~ /<BOOT_BIN>/)
			{
				$readLine =~ s/<BOOT_BIN>/$binaryFile/;
			}			
			print hTTMacroTemplate ($readLine);
		}
		close(hTTMacroTemplate);
		close(hTTMacroTemplateCopy);
	}
	# Update the Tera term script file with current binary file name
	{
		open (hTTMacroTemplate, "temp.txt") || die "***"."temp.txt"." file open failed ***\n";
		open (hTTMacrofile, ">".$TeraTermMacroFile) || die "***>".$TeraTermMacroFile." file open failed ***\n";
		
		until(eof(hTTMacroTemplate))
		{
			my $readLine = <hTTMacroTemplate>;
			if($readLine !~ /<BIN_FILE_NAME>/)
			{
				print hTTMacrofile ($readLine);
			}
			else
			{
				$readLine =~ s/<BIN_FILE_NAME>/$tableTestCases[$locLoopCount]{"testCaseId"}{"binaryName"}/;
				print hTTMacrofile ($readLine);
			}
			
		}
		close(hTTMacrofile);
		close(hTTMacroTemplate);
	}

	$logFileName = $tableTestCases[$locLoopCount]{"testCaseId"}{"logPath"}."/".$logFileNameTemplate;
	
	$logFileName =~ s/<LOGFILENAME>/$tableTestCases[$locLoopCount]{"testCaseId"}{"binaryName"}_TC$tableTestCases[$locLoopCount]{"testCaseId"}.log/;
	$logFileName =~ s/\//\\/g;

	$logFileNameTemp = $tableTestCases[$locLoopCount]{"testCaseId"}{"logPath"}."/".$logFileNameTemplate;
	$logFileNameTemp =~ s/<LOGFILENAME>/tempLogFile.txt/;
	$logFileNameTemp =~ s/\//\\/g;
	
	unlink($logFileNameTemp); 
	
	if(($tableTestCases[$locLoopCount]{"testCaseId"}{"scriptName"}!~/[a-z]/i)&&($tableTestCases[$locLoopCount]{"testCaseId"}{"scriptName"}!~/NA/))
	{
		my $fptrExecTC = $tableTestCases[$locLoopCount]{"testCaseId"}{"scriptName"};
		
		$cmd = "ttermpro.exe /C=1 /F=".$TeraTermConfigFile." /M=".$TeraTermMacroFile." /L=".$logFileNameTemp."\n";
		eval
		{
			# wait for the timeout period specified for each application before timing out.
			alarm($tableTestCases[$locLoopCount]{"testCaseId"}{"timeOut"}+30);
			print "Loading BootLoader\n\n\n";

			$pid = system 1, $cmd;

			sleep($tableTestCases[$locLoopCount]{"testCaseId"}{"timeOut"});
			alarm(0);
			
		};	
		
		# timed out
#		if ($@ =~ /TIMED OUT/ )
#		{
#			system("taskkill /im ttpmacro.exe");
#			print "TIMED OUT!!!!!\n\n\n";
#		}
#		else
#		{
#			print "DID NOT TIMED OUT ;-)\n\n\n";
#		}
		
		print "Test case completed for ".$tableTestCases[$locLoopCount]{"testCaseId"}{"binaryName"}."\n";
		
		analyze_default($locLoopCount);
	}
	else
	{
		my $fptrExecTC = $tableTestCases[$locLoopCount]{"testCaseId"}{"scriptName"};
		
		$fptrExecTC =~ s/\.pm//;
		&$fptrExecTC($locLoopCount);
	}

	system("taskkill /im ttermpro.exe");
	
	$cmd = "staf 192.168.247.107 usbremotepower off 1";
	system ($cmd);
	
	open (hTCStatusFile, ">>".$rootpath."/".$TCStatusFileName) || die "***".$TCStatusFileName." file open failed ***\n";
	print hTCStatusFile ("\n");
	print hTCStatusFile ($tableTestCases[$locLoopCount]{"testCaseId"}.",");
	print hTCStatusFile ($tableTestCases[$locLoopCount]{"testCaseId"}{"binaryName"}.",");
	print hTCStatusFile ($tableTestCases[$locLoopCount]{"testCaseId"}{"scriptName"}.",");
	print hTCStatusFile ($tableTestCases[$locLoopCount]{"testCaseId"}{"timeOut"}.",");
	print hTCStatusFile ($tableTestCases[$locLoopCount]{"testCaseId"}{"testStatus"}.",");
	print hTCStatusFile ($tableTestCases[$locLoopCount]{"testCaseId"}{"failCondition"}.",");
	print hTCStatusFile ($tableTestCases[$locLoopCount]{"testCaseId"}{"Remarks"});
	close(hTCStatusFile);

	sleep(6);
	unlink($logFileNameTemp);
}
unlink($rootpath."/temp.txt"); 
close(hSourceFile);
exit(0);
