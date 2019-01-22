#!/usr/bin/perl

print "MsgId, DevType, Mfg, API, DevNum, Data\n";
while (my $line = <STDIN>)
{
	if ($line =~ /can0\s+([0-9A-Fa-f]{8})\s+\[\d\]\s+(.+)/)
	{
		my $msgId = hex($1);
		my $devType = ($msgId >> 24) & 0x0000001f;
		my $mfg = ($msgId >> 16) & 0x000000ff;
		my $api = ($msgId >> 6) & 0x0000007ff;
		my $devNum = $msgId & 0x0000003f;
		print "$1,$devType,$mfg,$api,$devNum,$2\n";
	}
}
