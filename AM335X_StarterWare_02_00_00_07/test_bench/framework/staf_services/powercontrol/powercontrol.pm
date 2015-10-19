#############################################################################
# Software Testing Automation Framework (STAF)                              #
# (C) Copyright IBM Corp. 2007                                              #
#                                                                           #
# This software is licensed under the Eclipse Public License (EPL) V1.0.    #
#############################################################################

package PowerControl;
use lib "D:/STAF/bin"; 
use Win32::SerialPort 0.19;
use PLSTAFService;
use PLSTAF;
use threads;
use threads::shared;
use Thread::Queue;

# In this queue the master threads queue jobs for the slave worker
my $work_queue = new Thread::Queue;
my $free_workers : shared = 0;
our $fServiceName;         # passed in part of parms
our $fHandle;               # staf handle for service
our $fPowerOnParser;         # parsers for different requests
our $fPowerOffParser;          # .
our $fHelpParser;
our $fLineSep;             # line separator
our $PortName;
our $PortObj;
our $switchNumber;
our $paramString;

sub new
{
    my ($class, $info) = @_;

    my $self =
    {
        threads_list => [],
        worker_created => 0,
        max_workers => 5, # do not create more than 5 workers
    };

    $fServiceName = $info->{ServiceName};
    $fHandle = STAF::STAFHandle->new("STAF/Service/" . $fServiceName);

	$paramString = $info->{Params};
	$paramString = lc($paramString);
	
	my @arr_param = split(/ /, $paramString);

	if ($arr_param[0] eq 'port')
	{
	  $PortName = $arr_param[1];
	}
	else
	{	
	  $PortName = $arr_param[1];
	  return (STAFResult::kInvalidRequestString,"Invalid PARMS: " . $arr_param[0]);
	}	

    # Power On parser
    $fPowerOnParser = STAFCommandParser->new();
    $fPowerOnParser->addOption("ON", 1, STAFCommandParser::VALUEREQUIRED);

    # Power Off parser
    $fPowerOffParser = STAFCommandParser->new();
    $fPowerOffParser->addOption("OFF", 1, STAFCommandParser::VALUEREQUIRED);
	
	# Help Parser
	$fHelpParser = STAFCommandParser->new();
    $fHelpParser->addOption("HELP", 1, STAFCommandParser::VALUENOTALLOWED);
	
    my $lineSepResult = $fHandle->submit2($STAF::STAFHandle::kReqSync,
        "local", "var", "resolve string {STAF/Config/Sep/Line}");

    $fLineSep = $lineSepResult->{result};
    return bless $self, $class;
}

sub AcceptRequest
{
    my ($self, $info) = @_;
    my %hash : shared = %$info;

    if ($free_workers <= 0 and
        $self->{worker_created} < $self->{max_workers})
    {
        my $thr = threads->create(\&Worker);
        push @{ $self->{threads_list} }, $thr;
        $self->{worker_created}++;
    }
    else
    {
        lock $free_workers;
        $free_workers--;
    }

    $work_queue->enqueue(\%hash);

    return $STAF::DelayedAnswer;
}

sub Worker
{
    my $loop_flag = 1;
    while ($loop_flag)
    {
        eval
        {
            # get the work from the queue
            my $hash_ref = $work_queue->dequeue();

            if (not ref($hash_ref) and $hash_ref->{request} eq 'stop')
            {
                $loop_flag = 0;
                return;
            }

            my ($rc, $result) = handleRequest($hash_ref);

            STAF::DelayedAnswer($hash_ref->{requestNumber}, $rc, $result);

            # increase the number of free threads
            {
                lock $free_workers;
                $free_workers++;
            }
        }
    }

    return 1;
}

sub handleRequest
{
    my $info = shift;
    my $lowerRequest = lc($info->{request});
    my $requestType = "";

    # get first "word" in request
    if($lowerRequest =~ m/\b(\w*)\b/)
    {
        $requestType = $&;
    }
    else
    {
        return (STAFResult::kInvalidRequestString,"Unknown PowerControlService Request: " . ($info->{request}));
    }

    if ($requestType eq "on")
    {
        return handlePowerOn($info);
    }
    elsif ($requestType eq "off")
    {
        return handlePowerOff($info);
    }
    elsif ($requestType eq "help")
    {
        return handleHelp($info);
    }	
    else
    {
        return (STAFResult::kInvalidRequestString,"Unknown PowerControlService Request: " . $info->{request});
    }

    return (0, "");
}


sub handlePowerOn
{
	my $info = shift;
	#my $switchOnCommand;

    if($info->{trustLevel} < 2)
    {
        return (STAFResult::kAccessDenied,
           "Trust level 2 required for Power ON request. Requesting " . 
           "machine's trust level: " . $info->{trustLevel});
    }

	my $result=(STAFResult::kOk, "");
    my $resultString = "";
    my $resolveResult;
    my $switchValue = "";
    # parse request
    my $parsedRequest = $fPowerOnParser->parse($info->{request});
	# check results of parse
    if($parsedRequest->{"rc"} != STAFResult::kOk)
    {
	   return (STAFResult::kInvalidRequestString, $parsedRequest->{errorBuffer});
    }

    # resolve the value after 'ON' if necessary
    $resolveResult = resolveVar($info->{isLocalRequest}, $parsedRequest->optionValue("on"), $info->{requestNumber});
							  
    # check results of resolve
    if ($resolveResult->{"rc"} != STAFResult::kOk)
    {
        return $resolveResult;
    }
	$switchValue = $resolveResult->{"result"};
	$switchNumber = "$switchValue.1\r\n";
	serialSetting();
	serialWrite($switchNumber);
    return (STAFResult::kOk, "$switchNumber");
 }

sub handlePowerOff 
{
	my $info = shift;
	my $switchOffCommand;
    if($info->{trustLevel} < 2)
    {
        return (STAFResult::kAccessDenied,
           "Trust level 2 required for ADD request. Requesting " . 
           "machine's trust level: " . $info->{trustLevel});
    }

    my $result=(STAFResult::kOk, "");
    my $resultString = "";
    my $resolveResult;
    my $switchValue = "";
    # parse request
    my $parsedRequest = $fPowerOffParser->parse($info->{request});

	# check results of parse
    if($parsedRequest->{"rc"} != STAFResult::kOk)
    {
	   return (STAFResult::kInvalidRequestString, $parsedRequest->{errorBuffer});
    }

    # resolve the value after 'OFF' if necessary
    $resolveResult = resolveVar($info->{isLocalRequest}, $parsedRequest->optionValue("off"), $info->{requestNumber});
							  
    # check results of resolve
    if ($resolveResult->{"rc"} != STAFResult::kOk)
    {
        return $resolveResult;
    }
	
	$switchValue = $resolveResult->{"result"};
	$switchNumber = "$switchValue.0\r\n";

	serialSetting();
	serialWrite($switchNumber);
	return (STAFResult::kOk, "$switchNumber");
}


sub handleHelp
{
    return (STAFResult::kOk,
          "RemotePowerController Service Help" . $fLineSep
          . $fLineSep . "ON     <Switch Number>" 
          . $fLineSep . "OFF    <Switch Number>");

}
 
sub serialSetting
{
	printf "Opening the serial port\n";
	$PortObj = new Win32::SerialPort ($PortName) || die "Can't open Serial Port $PortName: $^E\n";	
	$PortObj->baudrate(9600);
	$PortObj->parity("none");
	$PortObj->databits(8);
	$PortObj->stopbits(1);
	$PortObj->handshake("none");		
	$PortObj->write_settings;
	$PortObj->start();
	return (STAFResult::kOk, "");
}

sub serialWrite
{
	$PortObj->write($_[0]);
	$PortObj->close || die "failed to close";
	printf "Closing the serial port\n";
	return (STAFResult::kOk, "");
}
 
sub resolveVar
{
    my ($machine, $optionValue, $requestNumber) = @_;
    my $value = "";
    my $resolvedResult;

    # look for something starting with '{'
    if ($optionValue =~ m/^\{/)
    {
        $resolvedResult =
            $fHandle->submit2($machine, "var", "resolve request " . 
                $requestNumber . " string " . $optionValue);

        if ($resolvedResult->{rc} != 0)
        {
            return $resolvedResult;
        }

        $value = $resolvedResult->{result};
    }
    else
    {
        $value = $optionValue;
    }
    return STAF::STAFResult->new(STAFResult::kOk, $value);
}


sub DESTROY
{
    my ($self) = @_;

    # Ask all the threads to stop, and join them.
    for my $thr (@{ $self->{threads_list} })
    {
        $work_queue->enqueue('stop');
    }

    # perform any cleanup for the service here
	undef $PortObj;

    #Un-register the service handle
    $fHandle->unRegister();
}
1;