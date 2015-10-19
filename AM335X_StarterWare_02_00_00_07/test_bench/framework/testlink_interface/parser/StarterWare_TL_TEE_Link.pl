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

use YAML::XS;
use RPC::XML::Client;
use File::Copy;
use Win32; 
use Win32::Process;
use Cwd;

$TCStatusFileHeading = "\ntestCaseId,binaryName,scriptName,timeOut,testStatus,failCondition,Remarks\n";
$outFileRelPath = "framework/testlink_interface/parser";
$parserFileRelPath = "framework/testlink_interface/parser";
$translatorFileRelPath = "framework/testlink_interface/translator";
$tcStatusFileName = "TCResults.txt";

# Create a YAML file
use Cwd qw(abs_path);

chdir "..\parser";

my $config = do{local(@ARGV,$/)="D:\\perlTools\\automation\\test\\framework\\testlink_interface\\parser\\config.yml";<>};
my $configuration = Load $config;
my $rootDir_AutomationFramework = $configuration->{'pathFrameworkBase'};
my $testLinkServerIp = $configuration->{'testLinkServerIp'};
my $resultsFileName = $configuration->{'resultsFileName'};

my @teeInFile = `staf local var get shared var starterware\@TEE1/STAF/TEE/inFile`;
my $xmlInFile = $teeInFile[2];
print $xmlInFile . "\n";
my @test_id;
my $countTestCases = -1;

###############################################################################################
####################             Invoke Individual Functions         ##########################
###############################################################################################
&result_TL;
&getTestCaseId;
&ExecuteTestCase;

############################################################################################
#################### Parsing the Tee_request file for getting the test case IDs  ###########
############################################################################################	

sub getTestPlanId()
{
	my $search = 0;
	my $locTestPlanID = 0;
	open (TEE, $xmlInFile) || die $!;
	
	GETTESTCASEIDLOOP: until(eof(TEE))
	{
		$searchLine = <TEE>;
		chomp($searchLine);
		
		if(($searchLine !~ /<testplan>/)&&($search == 0))
		{
			next GETTESTCASEIDLOOP;
		}
		elsif(($searchLine =~ /<\/testplan>/)&&($search == 1))
		{
			$search = 0;
		}
		else
		{
			$search = 1;
		}

		if($searchLine =~ /<id>/)
		{
			#$searchLine =~ s/^[0-9]{7}$//g;
			$searchLine =~ s/\D//g;
			$locTestPlanID = $searchLine;
			last;
		}
	}
	close(TEE);	

	return $locTestPlanID;
}

############################################################################################
#################### parsing the Tee_request file for getting the Build IDs  ###############
############################################################################################	

sub getBuildId()
{
	my $search = 0;
	my $locBuildID = 0;
	open (TEE, $xmlInFile) || die $!;
	
	GETTESTCASEIDLOOP: until(eof(TEE))
	{
		$searchLine = <TEE>;
		chomp($searchLine);
		
		if(($searchLine !~ /<build>/)&&($search == 0))
		{
			next GETTESTCASEIDLOOP;
		}
		elsif(($searchLine =~ /<\/build>/)&&($search == 1))
		{
			$search = 0;
		}
		else
		{
			$search = 1;
		}

		if($searchLine =~ /<id>/)
		{
			#$searchLine =~ s/^[0-9]{7}$//g;
			$searchLine =~ s/\D//g;
			$locBuildID = $searchLine;
			last;
		}
	}
	close(TEE);	

	return $locBuildID;
}

############################################################################################
#################### Parsing the Tee_request file for getting the test case IDs  ###########
############################################################################################	

sub getTestCaseId()
{
	my $search = 0;
	$countTestCases = -1;
	open (TEE, $xmlInFile) || die $!;
	
	GETTESTCASEIDLOOP: until(eof(TEE))
	{
		$searchLine = <TEE>;
		chomp($searchLine);
		
		if(($searchLine !~ /<testcase/)&&($search == 0))
		{
			next GETTESTCASEIDLOOP;
		}
		elsif(($searchLine =~ /<\/testcase>/)&&($search == 1))
		{
			$search = 0;
		}
		else
		{
			$search = 1;
		}

		if($searchLine =~ /<id>/)
		{
			#$searchLine =~ s/^[0-9]{7}$//g;
			$searchLine =~ s/\D//g;
			print $searchLine."\n\n\n";
			$countTestCases++;
			$test_id[$countTestCases] = $searchLine;
		}
	}
	close(TEE);	
}


###################################################################################################################
### This function invokes VATF Lite and executes test cases one by one and update the result in TestLink Server ###	
###################################################################################################################

sub ExecuteTestCase()
{	
	my $locLoopCount = 0;
	my $TestStatus = "";
	my $TestComment = "";
	my $PlanId = &getTestPlanId;
	my $BuildId = &getBuildId;
	
	my $parserFilePath = $rootDir_AutomationFramework."/".$parserFileRelPath."/";
	
	################################################################################################################
	### Create run.bat file.
	################################################################################################################
	
	my $drive = "";
	my $tempIndex = 0;
	my $execDir = $rootDir_AutomationFramework;

	$execDir =~ s/\//\\\\/g;

	$tempIndex = rindex($execDir, ":");;
	$drive = substr ($execDir, ($tempIndex-1), 2);
	
	open (hBatFile, ">".$parserFilePath."run.bat") || die "***>".$parserFilePath."run.bat"." file open failed ***\n";
	print hBatFile ("cd $execDir"."\n");
	print hBatFile ($drive."\n");
	print hBatFile ("perl StarterWareTB.pl\n");
	close(hBatFile);

	open (TEE, $xmlInFile) || die "***".$xmlInFile." file open failed ***!!!\n";

	until(eof(TEE))
	{
		$searchLine = <TEE>;
		chomp($searchLine);
		@searchLineWords = split(/\s+/, $searchLine);
		$searchLineWordcount = @searchLineWords;
		
		if($searchLine =~ /<params_control>/)
		{
			$searchLine =~ s/<params_control>//;
			chomp($searchLine);
			if(($searchLine =~ /CDATA/)&&($searchLine =~ /\.bin/))
			{
				$searchLine =~ s/<!\[CDATA\[//;
				$searchLine =~ s/[[:cntrl:]]+//;
				$searchLine =~ s/^\s+//;
				
				open (hDestFile, ">".$parserFilePath."listTestCases.txt") || die "***>".$parserFilePath."listTestCases.txt file open failed ***\n";
				print hDestFile ($searchLine."\n");
				print $searchLine."\n";
				close(hDestFile);
				
				sleep(10);
				my $exe_batch = `staf local process start shell sameconsole command $parserFilePath\/run.bat Focus Foreground wait stderrtostdout returnstdout`;
				print $exe_batch;
				
				open (ResultFile, $rootDir_AutomationFramework."/".$tcStatusFileName) || die "***".$rootDir_AutomationFramework."/".$tcStatusFileName." file open failed ***!!!\n";

				RESULTFILELOOP: until(eof(ResultFile))
				{
					$searchLine = <ResultFile>;
					chomp($searchLine);
					
					if(($searchLine !~ /[A-Za-z0-9_]/)||($searchLine eq $TCStatusFileHeading))
					{
						next RESULTFILELOOP;
					}
					
					if(($searchLine =~ /Pass/)||($searchLine =~ /Application Passed/))
					{
						$TestStatus = "p";
						$TestComment = "";
					}
					else
					{
						$TestStatus = "f";
						if($searchLine =~ /TIMED OUT/)
						{
							$TestComment = "TIMED OUT";
						}
						else
						{
							$TestComment = "Application Error";
						}
					}
				}
				close(ResultFile);

				$cmd = "$testLinkServerIp TLWriter update planid $PlanId buildid $BuildId testcase $test_id[$locLoopCount] status $TestStatus notes \"$TestComment\"";
				print $cmd;
				my $stafResult = `staf $cmd`;		
				print $stafResult;	
				print "\n";	
				$locLoopCount++;
				
				if($locLoopCount > $countTestCases)
				{
					last;
				}
			}
		}
	}
	close(TEE);	
}


sub result_TL()
{
	$srcfile = $rootDir_AutomationFramework."/".$translatorFileRelPath."/"."automation_results_template.xml";
	$destifile = $rootDir_AutomationFramework."/".$translatorFileRelPath."/".$resultsFileName;
	
	$srcfile =~ s/\//\\\\/g;
	$destifile =~ s/\//\\\\/g;
	
    print "\n  DEBUG :: This is dummy file copy to avoid JAVA error \n";
	copy($srcfile,$destifile) or die "Copy failed: $!";
    my $string =`staf  local fs copy file $srcfile tofile $destifile tomachine local`;
    print $string;
                
}

