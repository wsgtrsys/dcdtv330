run ()
{
	echo $@
	$@
	if [ "$?" != "0" ]; then
		echo "*** ERROR. Aborting ***"
		exit -1
	fi
}

echo "Checking if the needed tools are present"
run ./check_needs.pl
echo
echo "************************************************************"
echo "* This script will download the latest tarball and build it*"
echo "* Assuming that your kernel is compatible with the latest  *"
echo "* drivers. If not, you'll need to add some extra backports,*"
echo "* ./backports/<kernel> directory.                          *"
echo "* It will also update this tree to be sure that all compat *"
echo "* bits are there, to avoid compilation failures            *"
echo "************************************************************"
echo
sleep 5
run git pull git://linuxtv.org/media_build.git master

run make -C linux/ download
run make -C linux/ untar
run make

echo "**********************************************************"
echo "* Compilation finished. Use "make install" to install them"
echo "**********************************************************"

