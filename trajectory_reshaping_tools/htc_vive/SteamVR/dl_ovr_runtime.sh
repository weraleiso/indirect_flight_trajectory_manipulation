#!/bin/bash

OS=`uname`

case $OS in
"Darwin")
	open steam://startvrmode
	;;
"Linux")
	xdg-open steam://startvrmode
	;;
*)
	echo "Unknown platform."
	exit -1
	;;
esac

