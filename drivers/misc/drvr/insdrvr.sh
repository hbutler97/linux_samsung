#!/bin/sh

#----------------------------------------------------------------------
# This shell script installs the the ADI PCIe  Driver Module
# into the Linux kernel.
#
# The major number is dynamically assigned.
# This will be mapped into /dev via udev rules

# dynamically assigned Major number for adipcie.ko
# Minor nmbers identify the board type, instance and BAR.
#----------------------------------------------------------------------

module="adipcie"

if [ `whoami` != root ]; then
	echo "ERROR! Must be root to install driver."
	exit 1
fi

echo "Installing driver module "${module}" into kernel."

# Install the driver module (pass any command line args - none expected)
/sbin/insmod -f ./${module}.ko $* || exit 1


echo "Done."

