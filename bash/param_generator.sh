#!/bin/bash

echo $#

if [ $# -ne 1 ]
then
    echo "Usage: $0 filename"
else
    cat "$1" |\
        sed 's,^\s*~\([^ ]*\)\s.*default: \([^)]*\)).*,<param="\1" value="\2" />\n<!-- & -->,' |\
        sed 's,^\s*[a-zA-Z(].*,<!-- & -->,' |\
        sed 's,^\s*~\([^ ]*\)\s.*,<!-- param="\1" /> -->\n<!-- & -->,' |\
        sed 's,"",",g' |\
        tee "$1.xml"
fi
