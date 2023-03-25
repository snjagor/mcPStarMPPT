#!/bin/bash
buff=0;
togB="";
#--possible inputs: logs:hrmtr,start addr. profiles:testing. 
while [ -n "$1" ]; do
	if [ "$1" = "-b" ]; then 
		togB=""; #"-b"; Off !  (unused opt)
	elif (( ($1 > 0) && ($1 < 255) )); then
		buff="$1";
	fi
	shift; 
done
i=$buff; x=0;
cmd0="./mcPStarMPPT"; #--:cmd  debug

echo;echo starting testing - with buffer of "$buff";
#----------------------------------------
cmd="$cmd0 -h"; $cmd; echo;echo '-----------------------------------------------------';echo;echo; #-

cmd="$cmd0 poll"; $cmd; echo;echo;echo; #-

cmd="$cmd0"; 
echo "cmd: $cmd"; $cmd; echo;echo;echo; #-

cmd="$cmd0 debug"; 
echo "##---cmd: $cmd ---"; $cmd; echo;echo '-----------------------------------------------------';echo;echo; #-

cmd="$cmd0 eeprom"; 
echo "##---cmd: $cmd ---"; $cmd; echo;echo '-----------------------------------------------------';echo;echo; #-

cmd="$cmd0 raw eeprom"; 
echo "##---cmd: $cmd ---"; $cmd; echo;echo '-----------------------------------------------------';echo;echo; #-

##--silent:
#cmd="$cmd0 -s"; 
#echo "##---cmd: $cmd ---"; $cmd; echo;echo '-----------------------------------------------------';echo;echo; #-

##--output to files:
cmd="$cmd0 json+"; 
echo "##---cmd: $cmd ---"; $cmd; echo;echo '-----------------------------------------------------';echo;echo; #-
cmd="$cmd0 json poll"; 
echo "##---cmd: $cmd ---"; $cmd; echo;echo '-----------------------------------------------------';echo;echo; #-
cmd="$cmd0 json logs 2 days 1"; 
echo "##---cmd: $cmd ---"; $cmd; echo;echo '-----------------------------------------------------';echo;echo; #-

##--logs: +hourmeter, 
cmd="$cmd0 logs 2 days 1"; 
echo "##---cmd: $cmd ---"; $cmd; echo;echo '-----------------------------------------------------';echo;echo; #-

cmd="$cmd0 debug logs 3 days 2"; 
echo "##---cmd: $cmd ---"; $cmd; echo;echo '-----------------------------------------------------';echo;echo; #-

cmd="$cmd0 debug raw logs 2 days 2"; 
echo "##---cmd: $cmd ---"; $cmd; echo;echo '-----------------------------------------------------';echo;echo; #-

cmd="$cmd0 debug logs 2 start 0x8FF0"; 
echo "##---cmd: $cmd ---"; $cmd; echo;echo '-----------------------------------------------------';echo;echo; #-

##--bulk read tests:
cmd="$cmd0 debug logs 9 days 8"; 
echo "##---cmd: $cmd ---"; $cmd; echo;echo '-----------------------------------------------------';echo;echo; #-

cmd="$cmd0 debug logs 2 days 203"; 
echo "##---cmd: $cmd ---"; $cmd; echo;echo '-----------------------------------------------------';echo;echo; #-

##--profiles: create, validate, backup, ...test
cmd="$cmd0 debug profile create"; 
echo "##---cmd: $cmd ---"; $cmd; echo;echo '-----------------------------------------------------';echo;echo; #-
cat ../profiles/new.txt

cmd="$cmd0 debug raw profile create"; 
echo "##---cmd: $cmd ---"; $cmd; echo;echo '-----------------------------------------------------';echo;echo; #-
cat ../profiles/new.txt

cmd="$cmd0 profile create"; #testing
echo "##---cmd: $cmd ---"; $cmd; echo;echo '-----------------------------------------------------';echo;echo; #-
#mv ../profiles/new.txt ../profiles/testNow.txt
#...uncomment & change some test value...
cmd="$cmd0 profile validate new"; #testing
echo "##---cmd: $cmd ---"; $cmd; echo;echo '-----------------------------------------------------';echo;echo; #-
##--update:... after updates skip RAM? print just_EEPROM & display?

##--coils: +EQ test
cmd="$cmd0 reset_charge"; 
echo "##---cmd: $cmd ---"; $cmd; echo;echo '-----------------------------------------------------';echo;echo; #-

##--utils:
cmd="$cmd0 convert 15.567"; 
echo "##---cmd: $cmd ---"; $cmd; echo;echo '-----------------------------------------------------';echo;echo; #-

cmd="$cmd0 convert f16 19401"; 
echo "##---cmd: $cmd ---"; $cmd; echo;echo '-----------------------------------------------------';echo;echo; #-


#----------------------------------------
#buff=$(( buff + 7 ));
#cmd="$cmd0 logs 2 days $buff"; 
#echo "cmd: $cmd"; $cmd; echo;echo;echo; #-


