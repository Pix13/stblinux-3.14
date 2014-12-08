#!/bin/sh

echo "*** NOTE: 'cmatest' has been renamed to 'cmatool'. ***"
echo "The name 'cmatest' is deprecated and will be removed in a few releases."
echo "Please update your scripts accordingly."
echo

cmatool "$@"
