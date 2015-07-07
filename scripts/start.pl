#! /usr/bin/perl


# Super simple script. Determines if ROS is running, and if it is, sends 
# a true message on keyframeReset, which tells any homography calculation
# nodes running to start calculating.
use warnings;
use strict;

# See if roscore is running. 
my $rosRunning = `ps aux | grep roscore`;
if ($rosRunning !~ 'python'){
	print "ROS is not running.\n";
	exit();
}

my $startTopic = `rosparam get /startTopic`;
chomp ($startTopic);

if ($startTopic ne ""){
	my $commandStart = system("rostopic pub -1 /$startTopic std_msgs/Bool \"data: true\"");
}else{
	print "Start topic isn't defined yet.\n";
}
