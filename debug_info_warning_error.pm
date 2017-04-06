
package debug_info_warning_error;
our $VERSION = '1.00';
use strict;
use warnings;
use base 'Exporter';
our @EXPORT = qw(debug debug2 info warning error);

our $verbosity = 1;

sub info {
	my ($message) = @_;
	if ($verbosity >= 1) {
		print $message . "\n";
	}
}

sub debug {
	my ($message) = @_;
	if ($verbosity >= 2) {
		print "  DEBUG: " . $message . "\n";
	}
}

sub debug2 {
	my ($message) = @_;
	if ($verbosity >= 3) {
		print " DEBUG2: " . $message . "\n";
	}
}

sub warning {
	my ($message) = @_;
	if ($verbosity >= 1) {
		print "WARNING: " . $message . "\n";
	}
}

sub error {
	my ($message) = @_;
	if (! defined $message) {
		$message = "";
	}
	if ($verbosity >= 0) {
		print "  ERROR: " . $message . "\n";
	}
}

