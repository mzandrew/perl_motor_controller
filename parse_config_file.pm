# 2012-05-09 mza

package parse_config_file;
our $VERSION = '1.00';
use strict;
use warnings;
use base 'Exporter';
use lib '/home/pilas/build/motor';
use debug_info_warning_error;
our @EXPORT = qw(parse_config_file write_config_file @filter);

our @filter;
my $filename;

sub parse_config_file {
	($filename) = @_;
	debug("attempting to parse config file \"$filename\"...");
	if (! -e $filename) {
		debug("config file \"$filename\" not found");
		return;
	}
	open(CONFIGFILE, $filename) || return;
	info("parsing config file \"$filename\"...");
	while (my $line = <CONFIGFILE>) {
		chomp($line);
		$line =~ s/#.*//;
		next if ($line =~ /^$/);
		debug2("\"$line\"");
		if ($line =~ /filter_([0-9])[ \t]*=[ \t]*([.0-9]+)/) {
			$filter[$1] = $2;
			info("filter[$1] = $filter[$1]");
		#} elsif () {
		} else {
			debug("unrecognized line \"$line\" in config file \"$filename\"");
		}
	}
	close(CONFIGFILE);
}

sub write_config_file {
	($filename) = @_;
	open(CONFIGFILE, ">$filename") || return;
	# write file here...
	for (my $i=1; $i<=5; $i++) {
		my $string = "filter_$i = $filter[$i]";
		debug2($string);
		print CONFIGFILE "$string\n";
	}
	close(CONFIGFILE);
}

sub test {
	parse_config_file(".config");
	write_config_file(".config2");
	parse_config_file(".config2");
}

END {
	if (defined $filename && $filename !~ /^$/) {
		info("parse_config_file's END block was called");
		write_config_file($filename);
	}
}

1;

