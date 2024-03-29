#!/usr/bin/perl
use strict;
##--UTF-8 Support------------------:
use Encode qw/encode decode/;
use open ':std', ':encoding(UTF-8)';
binmode(CONFIG, ":utf8");

#my $u = decode('UTF-8', "&theta; θ &#952;");
#print OUT $u;  exit(0); #:works 

my ($dir, $json, $html, $file, $out, $od) = "";   my $debug=1;
$html = $1 if ($0 =~ /^(.*\/)[^\/]+$/i);
$html = $html.'json_template.html';
$out = 'json-test.html';

if (!$ARGV[0]) { &help(); exit(); }
$file = $ARGV[0];
if ($file =~ /^(.*\/)([^\/]+\.json)$/i) { $json = $2; $dir = $1; $od=$dir; }
else { die ">no json input.\n"; } 
if ($ARGV[1] && -d $ARGV[1]) { $od = $ARGV[1]; }
$file = $json; #:page title
$json = $dir.$json;
$out = $od.$out; 

#-:get json data from file:
open (JSON, "<", "$json") or die "no json! [$json] $!\n"; 
$json = do { local $/; <JSON> };
close (JSON);
$json =~ s/\'/\\\'/g; #:escape quotes for inserting
$json =~ s/\\/\\\\/g; #:escape bkslashes too!? or json_stringify() fails.
$json =~ s/[\r\n]//g; #:shouldn't be any, do anyway.

#-:open HTML template:
open (TMPL, "<", "$html") or die "no template! [$html] $!\n"; 
#$html = <TMPL>; close (TMPL);

#-:open OUT file:
open (OUT, ">", "$out") or die "$!\n";

#-:search HTML template and insert json into js block:
while (<TMPL>) {
	s/%%file%%/$file/i;
	s/%%json%%/$json/i;
	#-:write new data html file:
	print OUT $_;
}
close (OUT) or die "$!\n";
close (TMPL);
print "$out\n";

exit(0);


sub help {
	my $fi = $0; $fi =~ s/.*(\/[^\/]+)$/\.$1/i;
	print "---------------------------------\n".
"Usage:
	$fi [json-file] [optional:alt output dir]
	  * default output is to same dir as json input. *
out file name: [$out]
**_ centerflowing.com _**\n";

}
