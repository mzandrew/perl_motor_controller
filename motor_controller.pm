# 2010-03-13 to 2010-03-17 mza @ uh idlab

# motor 1 is the delay
# motor 2 is the filter
# motor 3 is the 
# motor 4 is the 

my $measured_number_of_steps_for_extent_of_delay_slide  = 61835;
#my $measured_number_of_steps_for_extent_of_filter_slide = 21186;
#my $measured_number_of_steps_for_extent_of_x_slide      = 10989;
#my $measured_number_of_steps_for_extent_of_y_slide      = 10975;

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

package motor_controller;
our $VERSION = '1.00';
use strict;
use warnings;
use base 'Exporter';
our @EXPORT = qw(move_delay_slide_to_macrostep_position decrease_delay_slide_by_one_macrostep increase_delay_slide_by_one_macrostep current_delay_in_picoseconds current_delay_in_whole_picoseconds current_whole_macrostep_position_of_delay_slide bring_delay_slide_to_beginning delay_macrosteps_steps_and_picoseconds_from_macrosteps move_filter_slide_to_macrostep_position bring_filter_slide_to_beginning current_macrostep_position_of_filter_slide current_macrostep_position_of_delay_slide $delay_slide_granularity_in_picoseconds go_to_absolute_position_for_delay_slide move_delay_slide_to_this_delay_in_picoseconds absolute_macrosteps_to_absolute_picoseconds get_limits_of_motion move_xy_slide_to_granular_relative_position save_position_and_move_x_slide_out_of_way restore_position_of_x_slide move_xy_slide_to_granular_relative_position restore_x_and_y_positions get_all_original_motor_positions);

use Device::SerialPort;
use Time::HiRes qw(usleep);
#use lib 'dirname($ENV{0})';
#use lib '$ENV{HOME}/build/motor';
#use lib '/home/mza/build/motor';
use debug_info_warning_error;

my $serial_device = "/dev/ttyUSB0";
#my $serial_device = "/dev/ttyS0";

#$debug_info_warning_error::verbosity = 2;
$debug_info_warning_error::verbosity = 3;

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

sub numeric_value {
	my ($message, $value) = @_;
	my $padding = "";
	for (my $i=0; $i<47-length($message); $i++) {
		$padding .= " ";
	}
	return $padding . $message . ":  " . $value;
}

sub msleep {
	my ($milliseconds) = @_;
	debug2("waiting for " . $milliseconds . " ms...");
	my $microseconds = $milliseconds*1000;
	usleep($microseconds);
}

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

# preliminary calculations:
my $number_of_steps_per_revolution = 200;
my $number_of_threads_per_millimeter = 1;
my $number_of_millimeters_per_inch = 25.4;
my $number_of_threads_per_inch = $number_of_millimeters_per_inch / $number_of_threads_per_millimeter;
my $number_of_steps_per_millimeter = $number_of_steps_per_revolution;

my $should_actually_connect_to_motor_controller = 1;
my $should_actually_move_motors = 1; # whether to actually run any motor commands

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#$should_actually_move_motors = 0; # don't actually run any motor commands
my $delay_to_wait_for_responses_from_motor_controller_in_milliseconds_primary   =  20;
my $delay_to_wait_for_responses_from_motor_controller_in_milliseconds_secondary = 200;
my $should_reset_positions_on_bootup           =  0;
my $should_keep_original_delay_slide_position  =  0;
my $should_keep_original_filter_slide_position =  0;
my $should_keep_original_x_slide_position      =  0;
my $should_keep_original_y_slide_position      =  0;
our $delay_slide_granularity_in_picoseconds     = 50;
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

my $communication_channel;
my $needs_to_be_closed = 0;

#my $highest_position_for_motor_1 = 45;
#my $highest_position_for_motor_2 = 45;

my $original_filter_slide_position = 0;
my $original_delay_slide_position = 0;
my $original_x_slide_position = 0;
my $original_y_slide_position = 0;

my $modified_filter_slide_position = 0;
my $modified_delay_slide_position = 0;
my $modified_x_slide_position = 0;
my $modified_y_slide_position = 0;

#our $current_filter_slide_macrostep_position = 0;
#our $current_delay_slide_macrostep_position  = 0;
#our $delay_in_whole_picoseconds    = 0;

my $number_of_steps_between_filter_slide_positions = $number_of_steps_per_millimeter * 19.05; # 0.75 inches

my $index_of_refraction_of_air_for_405nm_light = 1.00029; # page 6 of http://arxiv.org/ftp/physics/papers/0502/0502100.pdf referencing G. Shortley and D. Williams, Elements of Physics (Prentice-Hall, Englewood Cliffs, N.J., 1961), p. 528.
my $speed_of_light_in_a_vacuum = 299792458; # in m/s from http://en.wikipedia.org/wiki/Speed_of_light
my $speed_of_light_in_air_at_405nm = $speed_of_light_in_a_vacuum / $index_of_refraction_of_air_for_405nm_light;

my $length_of_delay_slide_in_millimeters = $measured_number_of_steps_for_extent_of_delay_slide / $number_of_steps_per_millimeter;
#my $length_of_delay_slide_in_inches = 12; # this is not measured
#my $length_of_delay_slide_in_millimeters = $length_of_delay_slide_in_inches * $number_of_millimeters_per_inch; # 304.8
my $number_of_picoseconds_per_millimeter = 1.0e9 / $speed_of_light_in_air_at_405nm;
my $length_of_delay_slide_in_picoseconds = $length_of_delay_slide_in_millimeters * $number_of_picoseconds_per_millimeter;
my $number_of_steps_per_picosecond = $number_of_steps_per_millimeter / $number_of_picoseconds_per_millimeter;
my ($smallest_filter_macrostep_position, $largest_filter_macrostep_position) = (0, 4);
our ($smallest_delay_in_ps, $largest_delay_in_ps) = (0, $length_of_delay_slide_in_picoseconds);
my ($smallest_delay_in_macrosteps, $largest_delay_in_macrosteps) = (0, int($largest_delay_in_ps/$delay_slide_granularity_in_picoseconds));
my $number_of_millimeters_per_delay_slide_macrostep = $delay_slide_granularity_in_picoseconds / $number_of_picoseconds_per_millimeter;
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
my $number_of_steps_for_offset_of_filter_slide = $number_of_steps_per_millimeter * 20 - $number_of_steps_between_filter_slide_positions;
my $number_of_steps_for_offset_of_delay_slide  = $number_of_steps_per_millimeter * 0;
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

my $number_of_steps_per_delay_slide_macrostep = $number_of_steps_per_picosecond * $delay_slide_granularity_in_picoseconds;

 info(numeric_value(                     "number of threads per inch", $number_of_threads_per_inch));
 info(numeric_value(                 "number of steps per millimeter", $number_of_steps_per_millimeter));
 info(numeric_value( "number of steps between filter slide positions", $number_of_steps_between_filter_slide_positions));
 info(numeric_value(          "speed of light in air for 405nm light", $speed_of_light_in_air_at_405nm . " m/s"));
 info(numeric_value(           "length of delay slide in millimeters", $length_of_delay_slide_in_millimeters . " mm"));
 info(numeric_value(           "number of picoseconds per millimeter", $number_of_picoseconds_per_millimeter));
 info(numeric_value(           "length of delay slide in picoseconds", $length_of_delay_slide_in_picoseconds . " ps"));
 info(numeric_value(                 "number of steps per picosecond", $number_of_steps_per_picosecond));
 info(numeric_value(      "number of steps per delay slide macrostep", $number_of_steps_per_delay_slide_macrostep));
 info(numeric_value("number of millimeters per delay slide macrostep", $number_of_millimeters_per_delay_slide_macrostep . " mm"));
# info(numeric_value("", $));

print "\n";
if (!$should_actually_connect_to_motor_controller) {
	warning("faking connection to the motor controller");
}
if (!$should_actually_move_motors) {
	warning("faking the motor control commands");
}
print "\n";

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

sub open_communications_port {
	debug("attempting to open communications device \"$serial_device\"...");
	$communication_channel = new Device::SerialPort($serial_device) || die "\nerror:  Cannot connect to motor controller.\nIs the motor controller connected and powered on?\n";
	$needs_to_be_closed = 1;
	$communication_channel->databits(8);
	$communication_channel->baudrate(9600);
	$communication_channel->parity("none");
	$communication_channel->stopbits(1);
	debug("switching motor controller to online mode");
	write_to_motor_controller("FCV");
	msleep(150);
	read_from_motor_controller(2);
}

sub close_communications_port {
	debug("switching motor controller to offline mode");
	write_to_motor_controller("Q");
	msleep(150);
	$communication_channel->close();
}

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

sub quit {
	my ($level) = @_;
	if ($needs_to_be_closed) {
		$needs_to_be_closed = 0;
		close_communications_port();
	}
	die("exiting with errorlevel $level");
#	exit($level);
}

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

sub read_number_from_motor_controller {
	if ($should_actually_connect_to_motor_controller) {
		my ($number_of_bytes) = @_;
		return read_from_motor_controller($number_of_bytes);
	} else {
		return "666";
	}
}

sub read_from_motor_controller {
	my ($number_of_bytes) = @_;
	if ($should_actually_connect_to_motor_controller) {
		my $response = $communication_channel->read($number_of_bytes);
		chomp($response);
		$response =~ s/\r//;
		if (!$response) {
			error("no response from motor controller");
		} else {
			debug2("response from motor controller is \"" . $response . "\"");
		}
		return $response;
	} else {
		return "R";
	}
}

sub write_drive_command_to_motor_controller {
	my ($command) = @_;
	if ($should_actually_move_motors) {
		write_to_motor_controller($command);
	} else {
		debug("executing FAKE command \"$command\"");
	}
}

sub write_to_motor_controller {
	my ($command) = @_;
	if ($command =~ /^V$/) {
		debug2("executing command \"$command\"");
	} else {
		debug("executing command \"$command\"");
	}
	if ($should_actually_connect_to_motor_controller) {
		$communication_channel->write($command . "\n");
	}
}

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

sub check_if_motor_controller_is_plugged_in_and_on {
	write_to_motor_controller("V");
	msleep($delay_to_wait_for_responses_from_motor_controller_in_milliseconds_primary);
	my $response = read_from_motor_controller(2);
	if (!$response) {
		error();
		error("motor controller did not respond.");
		error("is it powered on?");
		error();
		quit(8);
	}
}

sub wait_until_motor_controller_is_ready {
	debug("waiting for motor controller to become ready");
	write_to_motor_controller("V");
	msleep($delay_to_wait_for_responses_from_motor_controller_in_milliseconds_primary);
	my $response = read_from_motor_controller(2);
	debug("waiting for motor controller to become ready");
	until($response =~ /R/) {
		write_to_motor_controller("V");
		msleep($delay_to_wait_for_responses_from_motor_controller_in_milliseconds_secondary);
		$response = read_from_motor_controller(2);
#		if ($response =~ /R/) { print "ready\n"; } elsif ($response =~ /B/) { print "busy\n"; }
	}
}

sub get_absolute_position_of_motor {
	my ($motor_number) = @_;
	my $response;
	     if ($motor_number == 1) {
		write_to_motor_controller("X");
	} elsif ($motor_number == 2) {
		write_to_motor_controller("Y");
	} elsif ($motor_number == 3) {
		write_to_motor_controller("Z");
	} elsif ($motor_number == 4) {
		write_to_motor_controller("T");
	} else {
		error("invalid motor number \"$motor_number\"");
	}
	msleep($delay_to_wait_for_responses_from_motor_controller_in_milliseconds_primary);
	my $maybe = read_from_motor_controller(1);
	if ($maybe =~ /^\^$/) {
		$response = read_number_from_motor_controller(10);
	} else {
		$response = $maybe . read_number_from_motor_controller(10);
	}
	return int($response);
}

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# absolute

# 0steps   0ps,0macrosteps    50ps,1macrostep
# |--------|------------------|------------------|------------------|------------------|----|

sub absolute_picoseconds_to_absolute_steps {
	my ($picoseconds) = @_;
	my $steps = relative_picoseconds_to_relative_steps($picoseconds) + $number_of_steps_for_offset_of_delay_slide;
	return $steps;
}

sub absolute_steps_to_absolute_picoseconds {
	my ($steps) = @_;
	my $picoseconds = relative_steps_to_relative_picoseconds($steps - $number_of_steps_for_offset_of_delay_slide);
	return $picoseconds;
}

sub absolute_macrosteps_to_absolute_steps {
	my ($macrosteps) = @_;
	my $steps = relative_macrosteps_to_relative_steps($macrosteps) + $number_of_steps_for_offset_of_delay_slide;
	return $steps;
}

sub absolute_steps_to_absolute_macrosteps {
	my ($steps) = @_;
	my $macrosteps = relative_steps_to_relative_macrosteps($steps - $number_of_steps_for_offset_of_delay_slide);
	return $macrosteps;
}

sub absolute_picoseconds_to_absolute_macrosteps {
	my ($picoseconds) = @_;
	my $macrosteps = relative_picoseconds_to_relative_macrosteps($picoseconds);
	return $macrosteps;
}

sub absolute_macrosteps_to_absolute_picoseconds {
	my ($macrosteps) = @_;
	my $picoseconds = relative_macrosteps_to_relative_picoseconds($macrosteps);
	return $picoseconds;
}


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# relative

sub relative_picoseconds_to_relative_steps {
	my ($picoseconds) = @_;
	my $steps = $picoseconds * $number_of_steps_per_picosecond;
	return $steps;
}

sub relative_steps_to_relative_picoseconds {
	my ($steps) = @_;
	my $picoseconds = $steps / $number_of_steps_per_picosecond;
	return $picoseconds;
}

sub relative_macrosteps_to_relative_steps {
	my ($macrosteps) = @_;
	my $steps = $macrosteps * $number_of_steps_per_delay_slide_macrostep;
	return $steps;
}

sub relative_steps_to_relative_macrosteps {
	my ($steps) = @_;
	my $macrosteps = $steps / $number_of_steps_per_delay_slide_macrostep;
	return $macrosteps;
}

sub relative_picoseconds_to_relative_macrosteps {
	my ($picoseconds) = @_;
	my $macrosteps = relative_steps_to_relative_macrosteps(relative_picoseconds_to_relative_steps($picoseconds));
	return $macrosteps;
}

sub relative_macrosteps_to_relative_picoseconds {
	my ($macrosteps) = @_;
	my $picoseconds = relative_steps_to_relative_picoseconds(relative_macrosteps_to_relative_steps($macrosteps));
	return $picoseconds;
}

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

sub current_position_of_delay_slide {
	my $steps = - get_absolute_position_of_motor(1);
	return $steps;
}

sub current_macrostep_position_of_delay_slide {
	my $steps = current_position_of_delay_slide();
	my $macrosteps = absolute_steps_to_absolute_macrosteps($steps);
	return $macrosteps;
}

sub current_whole_macrostep_position_of_delay_slide {
	my $macrosteps = int(0.5 + current_macrostep_position_of_delay_slide());
	return $macrosteps;
}

sub current_delay_in_picoseconds {
	my $steps = current_position_of_delay_slide();
	my $delay = absolute_steps_to_absolute_picoseconds($steps);
	return $delay;
}

sub current_delay_in_whole_picoseconds {
	my $delay = int(0.5 + current_delay_in_picoseconds());
	return $delay;
}

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

sub move_delay_slide_to_this_delay_in_picoseconds {
	my ($desired_delay_in_picoseconds) = @_;
	my $desired_steps = absolute_picoseconds_to_absolute_steps($desired_delay_in_picoseconds);
	if ($smallest_delay_in_ps <= $desired_delay_in_picoseconds && $desired_delay_in_picoseconds <= $largest_delay_in_ps) {
		go_to_absolute_position_for_delay_slide(-$desired_steps);
	} else {
		error("illegal delay slide position ($desired_delay_in_picoseconds ps => $desired_steps steps)");
	}
}

sub bring_delay_slide_to_beginning {
	print "bringing delay slide to beginning position to find end of slide\n";
	write_drive_command_to_motor_controller("CIA1M-0,R");
	write_drive_command_to_motor_controller("CI1M0,R");
	wait_until_motor_controller_is_ready(); 
	$original_delay_slide_position = - get_absolute_position_of_motor(1);
	info(numeric_value("original position of delay slide", $original_delay_slide_position));
	write_drive_command_to_motor_controller("CIA1M-0,R");
	if ($should_keep_original_delay_slide_position) {
		go_to_macrostep_position_for_delay_slide($original_delay_slide_position);
		show_delay();
	} else {
		move_delay_slide_to_macrostep_position(0);
	}
}

sub go_to_absolute_position_for_delay_slide {
	my ($absolute_position) = @_;
	write_drive_command_to_motor_controller("CIA1M" . int($absolute_position) . ",R");
	wait_until_motor_controller_is_ready(); 
}

sub delay_macrosteps_steps_and_picoseconds_from_macrosteps {
	my ($macrosteps) = @_;
	my $string = "[" . $macrosteps . ":" . int(0.5 + absolute_macrosteps_to_absolute_steps($macrosteps)) . "] ~ " . absolute_macrosteps_to_absolute_picoseconds($macrosteps) . " ps";
	return $string;
}

sub move_delay_slide_to_macrostep_position {
	my ($new_macrostep_position) = @_;
	#info(numeric_value("current delay slide position", delay_macrosteps_steps_and_picoseconds_from_macrosteps(current_whole_macrostep_position_of_delay_slide())));
	info(numeric_value("current delay slide position", delay_macrosteps_steps_and_picoseconds_from_macrosteps(current_macrostep_position_of_delay_slide())));
	info(numeric_value( "proposed new slide position", delay_macrosteps_steps_and_picoseconds_from_macrosteps($new_macrostep_position)));
	if ($smallest_delay_in_macrosteps <= $new_macrostep_position && $new_macrostep_position <= $largest_delay_in_macrosteps) {
		info(numeric_value("moving delay slide to new position", delay_macrosteps_steps_and_picoseconds_from_macrosteps($new_macrostep_position)));
		my $new_position_in_absolute_steps = - int(absolute_macrosteps_to_absolute_steps($new_macrostep_position));
		go_to_absolute_position_for_delay_slide($new_position_in_absolute_steps);
		show_delay();
	} else {
		error("illegal delay slide position");
	}
}

sub decrease_delay_slide_by_one_macrostep {
	move_delay_slide_to_macrostep_position(current_whole_macrostep_position_of_delay_slide()-1);
}

sub increase_delay_slide_by_one_macrostep {
	move_delay_slide_to_macrostep_position(current_whole_macrostep_position_of_delay_slide()+1);
}

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

sub bring_filter_slide_to_beginning {
	print "bringing filter slide to beginning position to find end of slide\n";
#	get_absolute_position_of_motor(2);
	write_drive_command_to_motor_controller("CIA2M-0,R");
#	get_absolute_position_of_motor(2);
	write_drive_command_to_motor_controller("CI2M0,R");
	wait_until_motor_controller_is_ready(); 
	$original_filter_slide_position = - get_absolute_position_of_motor(2);
	info(numeric_value("original position of filter slide", $original_filter_slide_position));
#	get_absolute_position_of_motor(2);
	write_drive_command_to_motor_controller("CIA2M-0,R");
#	get_absolute_position_of_motor(2);
	if ($should_keep_original_filter_slide_position) {
		go_to_absolute_position_for_filter_slide($original_filter_slide_position);
		wait_until_motor_controller_is_ready(); 
	}
#	get_absolute_position_of_motor(2);
}

sub go_to_absolute_position_for_filter_slide {
	my ($absolute_position) = @_;
	write_drive_command_to_motor_controller("CIA2M" . int($absolute_position) . ",R");
	wait_until_motor_controller_is_ready();
}

sub current_position_of_filter_slide {
	my $steps = - get_absolute_position_of_motor(2);
	return $steps;
}

sub current_macrostep_position_of_filter_slide {
	my $steps = current_position_of_filter_slide();
	my $macrosteps = filter_absolute_macrosteps_from_absolute_steps($steps);
	return $macrosteps;
}

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

# stop  offset     1          2          3          4
# |-----|----------|----------|----------|----------|----

sub filter_relative_macrosteps_from_relative_steps {
	my ($steps) = @_;
	my $macrosteps = $steps / $number_of_steps_between_filter_slide_positions;
	return $macrosteps;
}

sub filter_relative_steps_from_relative_macrosteps {
	my ($macrosteps) = @_;
	my $steps = $macrosteps * $number_of_steps_between_filter_slide_positions;
	return $steps;
}

sub filter_absolute_macrosteps_from_absolute_steps {
	my ($steps) = @_;
	my $macrosteps = filter_relative_macrosteps_from_relative_steps($steps - $number_of_steps_for_offset_of_filter_slide);
	if ($macrosteps < 1) {
		$macrosteps += filter_relative_macrosteps_from_relative_steps($number_of_steps_for_offset_of_filter_slide);
		$macrosteps *= ($number_of_steps_between_filter_slide_positions + $number_of_steps_for_offset_of_filter_slide) / $number_of_steps_between_filter_slide_positions;
	}
	return $macrosteps;
}

sub filter_absolute_steps_from_absolute_macrosteps {
	my ($macrosteps) = @_;
	my $steps = filter_relative_steps_from_relative_macrosteps($macrosteps);
	if ($macrosteps) {
		$steps += $number_of_steps_for_offset_of_filter_slide;
	}
	return $steps;
}

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

sub filter_macrosteps_and_steps_from_macrosteps {
	my ($macrosteps) = @_;
	my $string = "[" . $macrosteps . ":" . int(0.5 + filter_absolute_steps_from_absolute_macrosteps($macrosteps)) . "]";
	return $string;
}

sub move_filter_slide_to_macrostep_position {
	my ($new_macrostep_position) = @_;
	info(numeric_value("current filter slide macrostep position", filter_macrosteps_and_steps_from_macrosteps(current_macrostep_position_of_filter_slide())));
	info(numeric_value("proposed new slide macrostep position", filter_macrosteps_and_steps_from_macrosteps($new_macrostep_position)));
	if ($smallest_filter_macrostep_position <= $new_macrostep_position && $new_macrostep_position <= $largest_filter_macrostep_position) {
		info(numeric_value("moving filter slide to new macrostep position", filter_macrosteps_and_steps_from_macrosteps($new_macrostep_position)));
		#my $new_absolute_position = - int($number_of_steps_for_offset_of_filter_slide + ($new_macrostep_position) * $number_of_steps_between_filter_slide_positions);
		my $new_absolute_position = - int(filter_absolute_steps_from_absolute_macrosteps($new_macrostep_position));
		go_to_absolute_position_for_filter_slide($new_absolute_position);
#		$current_filter_slide_macrostep_position = $new_macrostep_position;
	} else {
		error("illegal filter slide position");
	}
}

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

sub go_to_relative_position_for_xy_slide {
	my ($delta_x, $delta_y) = @_;
	if ($delta_x) {
		write_drive_command_to_motor_controller("CI4M" . int($delta_x) . ",R");
		wait_until_motor_controller_is_ready();
		show_current_position_of_x_slide();
	}
	if ($delta_y) {
		write_drive_command_to_motor_controller("CI3M" . int($delta_y) . ",R");
		wait_until_motor_controller_is_ready();
	}
}

sub go_to_absolute_position_for_x_slide {
	my ($x) = @_;
	write_drive_command_to_motor_controller("CIA4M" . int($x) . ",R");
	wait_until_motor_controller_is_ready();
	show_current_position_of_x_slide();
}

sub go_to_motor_side_stop_for_x_slide {
	write_drive_command_to_motor_controller("CI4M-0,R");
	wait_until_motor_controller_is_ready();
	show_current_position_of_x_slide();
}

sub move_xy_slide_to_granular_relative_position {
	my ($delta_x, $delta_y) = @_;
#	print "($delta_x, $delta_y)\n";
#my $interstitial_distance_between_anodes_in_millimeters = 2.3;
my $interstitial_distance_between_anodes_in_millimeters = 0.25; # adjust granular x-y spacing here!
	$delta_x *= $interstitial_distance_between_anodes_in_millimeters * $number_of_steps_per_millimeter;
	$delta_y *= $interstitial_distance_between_anodes_in_millimeters * $number_of_steps_per_millimeter;
	$delta_x = - $delta_x;
	$delta_y = - $delta_y;
	go_to_relative_position_for_xy_slide($delta_x, $delta_y);
}

my $saved_position_of_x_slide;

sub show_current_position_of_x_slide {
	my $x = get_absolute_position_of_motor(4);
	info("current position of x slide:  $x");
}

sub save_position_and_move_x_slide_out_of_way {
	$saved_position_of_x_slide = get_absolute_position_of_motor(4);
	info("saved position of x slide:  $saved_position_of_x_slide");
	go_to_motor_side_stop_for_x_slide();
}

sub restore_position_of_x_slide {
	go_to_absolute_position_for_x_slide($saved_position_of_x_slide);
}

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

sub show_delay {
	info("current delay = " . current_delay_in_picoseconds() . " ps ~ " . current_delay_in_whole_picoseconds() . " ps");
}

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

sub bring_motor_to_endstop_near_motor {
	my ($motor_number) = @_;
	write_drive_command_to_motor_controller(motor_command_to_go_to_endstop_near_motor($motor_number));
	wait_until_motor_controller_is_ready();
	show_current_position_of_x_slide();
}

sub bring_motor_to_endstop_far_from_motor {
	my ($motor_number) = @_;
	write_drive_command_to_motor_controller(motor_command_to_go_to_endstop_far_from_motor($motor_number));
	wait_until_motor_controller_is_ready();
	show_current_position_of_x_slide();
}

sub bring_motor_to_absolute_position {
	my ($motor_number, $absolute_position) = @_;
	write_drive_command_to_motor_controller(motor_command_to_go_to_absolute_position($motor_number, $absolute_position));
	wait_until_motor_controller_is_ready();
	show_current_position_of_x_slide();
}

sub tell_motor_that_it_is_now_at_the_zero_position {
	my ($motor_number) = @_;
	show_current_position_of_x_slide();
	write_drive_command_to_motor_controller(motor_command_to_reset_the_zero($motor_number));
	wait_until_motor_controller_is_ready();
	show_current_position_of_x_slide();
}

sub motor_command_to_go_to_endstop_near_motor {
	my ($motor_number) = @_;
	my $string = "";
	if (1 <= $motor_number && $motor_number <= 4) {
		$string = "CI" . $motor_number . "M-0,R";
	} else {
	}
	return $string;
}

sub motor_command_to_go_to_endstop_far_from_motor {
	my ($motor_number) = @_;
	my $string = "";
	if (1 <= $motor_number && $motor_number <= 4) {
		$string = "CI" . $motor_number . "M0,R";
	} else {
	}
	return $string;
}

sub motor_command_to_go_to_absolute_position {
	my ($motor_number, $absolute_position) = @_;
	my $string = "";
	if (1 <= $motor_number && $motor_number <= 4) {
		if ($absolute_position != 0 || $absolute_position != int($absolute_position)) {
			$string = "CIA" . $motor_number . "M" . $absolute_position . ",R";
		}
	} else {
	}
	return $string;
}

sub motor_command_to_reset_the_zero {
	my ($motor_number) = @_;
	my $string = "";
	if (1 <= $motor_number && $motor_number <= 4) {
		$string = "CIA" . $motor_number . "M-0,R";
	} else {
	}
	return $string;
}

#	bring_motor_to_endstop_near_motor(4);
#	tell_motor_that_it_is_now_at_the_zero_position(4);
#	bring_motor_to_absolute_position(4, 4138);
#	bring_motor_to_endstop_far_from_motor(4);

sub get_limits_of_motion {
#	write_drive_command_to_motor_controller("CI1M0,IA1M-0,I2M0,IA2M-0,R");
#	write_drive_command_to_motor_controller("CI1M-0,I2M-0,R");
	wait_until_motor_controller_is_ready();

	write_drive_command_to_motor_controller("CI1M0,IA1M-0,R");
	wait_until_motor_controller_is_ready();
	info("beginning of travel for motor 1 = " . get_absolute_position_of_motor(1));
	write_drive_command_to_motor_controller("CI1M-0,R");
	wait_until_motor_controller_is_ready();
	info("      end of travel for motor 1 = " . get_absolute_position_of_motor(1));

	write_drive_command_to_motor_controller("CI2M0,IA2M-0,R");
	wait_until_motor_controller_is_ready();
	info("beginning of travel for motor 2 = " . get_absolute_position_of_motor(2));
	write_drive_command_to_motor_controller("CI2M-0,R");
	wait_until_motor_controller_is_ready();
	info("      end of travel for motor 2 = " . get_absolute_position_of_motor(2));
}

sub get_all_original_motor_positions {
	$original_delay_slide_position  = get_absolute_position_of_motor(1);
	$original_filter_slide_position = get_absolute_position_of_motor(2);
	$original_y_slide_position      = get_absolute_position_of_motor(3);
	$original_x_slide_position      = get_absolute_position_of_motor(4);
}

sub show_all_original_motor_positions {
	info("original position of delay slide  = " . $original_delay_slide_position);
	info("original position of filter slide = " . $original_filter_slide_position);
	info("original position of x slide      = " . $original_x_slide_position);
	info("original position of y slide      = " . $original_y_slide_position);
}

sub restore_x_and_y_positions {
	bring_motor_to_absolute_position(3, $original_y_slide_position);
	bring_motor_to_absolute_position(4, $original_x_slide_position);
}

sub save_all_positions_and_find_endstops_and_restore_positions {
	get_all_original_motor_positions();
	show_all_original_motor_positions();
	#bring_filter_slide_to_beginning();
	#bring_delay_slide_to_beginning();
	#go_to_motor_side_stop_for_y_slide();
	#go_to_motor_side_stop_for_x_slide(); this should contain the next line:
	#bring_motor_to_endstop_far_from_motor(1); # 1=
	#bring_motor_to_endstop_far_from_motor(2); # 2=
	#bring_motor_to_endstop_far_from_motor(3); # 3=
	bring_motor_to_endstop_near_motor(4); # 4=x
	$modified_delay_slide_position  = get_absolute_position_of_motor(1);
	$modified_filter_slide_position = get_absolute_position_of_motor(2);
	$modified_y_slide_position      = get_absolute_position_of_motor(3);
	$modified_x_slide_position      = get_absolute_position_of_motor(4);
	$original_delay_slide_position  =     ($original_delay_slide_position  - $modified_delay_slide_position);
	$original_filter_slide_position =     ($original_filter_slide_position - $modified_filter_slide_position);
	$original_x_slide_position      =  abs($original_x_slide_position      - $modified_x_slide_position);
	$original_y_slide_position      =     ($original_y_slide_position      - $modified_y_slide_position);
	show_all_original_motor_positions();
	#write_drive_command_to_motor_controller("CI1M-0,R");
	#write_drive_command_to_motor_controller("CI2M-0,R");
	#write_drive_command_to_motor_controller("CI3M-0,R");
	tell_motor_that_it_is_now_at_the_zero_position(4); # 4=x
	#go_to_absolute_position_for_delay_slide($original_delay_slide_position);
	#go_to_absolute_position_for_filter_slide($original_filter_slide_position);
	#go_to_absolute_position_for_y_slide($original_y_slide_position);
	bring_motor_to_absolute_position(4, $original_x_slide_position);
	#bring_motor_to_absolute_position(4, 4138);
}

sub initialize_positions_by_resetting_them_all {
	bring_filter_slide_to_beginning();
	bring_delay_slide_to_beginning();
}

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

sub test {
	if (0) {
		print "\n";
		info(relative_steps_to_relative_picoseconds(6000));
		info(relative_picoseconds_to_relative_steps(750));
		info(relative_steps_to_relative_macrosteps(6000));
		info(relative_macrosteps_to_relative_picoseconds(6));
		info(relative_picoseconds_to_relative_macrosteps(750));
		info(relative_macrosteps_to_relative_steps(3));
		print "\n";
		info(absolute_steps_to_absolute_picoseconds(6000));
		info(absolute_picoseconds_to_absolute_steps(750));
		info(absolute_steps_to_absolute_macrosteps(6000));
		info(absolute_macrosteps_to_absolute_picoseconds(6));
		info(absolute_picoseconds_to_absolute_macrosteps(750));
		info(absolute_macrosteps_to_absolute_steps(3));
		print "\n";
	}
	if (1) {
		info("current position of delay slide = " . current_position_of_delay_slide());
		info("current macrostep postion of delay slide = " . current_macrostep_position_of_delay_slide());
		#info("current delay = " . current_delay_in_whole_picoseconds() . " ps");
		#info("current delay = " . current_delay_in_picoseconds() . " ps");
		info("current delay = " . current_delay_in_picoseconds() . " ps ~ " . current_delay_in_whole_picoseconds() . " ps");
	}
}

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

INIT {
	open_communications_port();
	check_if_motor_controller_is_plugged_in_and_on();
#	wait_until_motor_controller_is_ready(); 
	if ($should_reset_positions_on_bootup) {
		initialize_positions_by_resetting_them_all();
	}
	#move_delay_slide_to_position(2);
	#move_delay_slide_to_position(1);
#	get_limits_of_motion(); exit(7);
#	show_all_current_motor_positions();
	#get_all_original_motor_positions();
	#save_all_positions_and_find_endstops_and_restore_positions();
	test();
}

END {
	info("the motor_control module's END block got called");
	close_communications_port();
}

1;

