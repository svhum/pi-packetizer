#!/bin/bash

# The following command will return two ptys, e.g. /dev/pts/X and /dev/pts/Y.
# The server program should be set to bind to the second for example, while the
# client to the first.

#socat -d -d pty,raw,echo=0,link=$HOME/dev/ttyS1 pty,raw,echo=0,link=$HOME/dev/ttyS2 2>&1 | grep dev
socat -d -d pty,raw,echo=0,link=$HOME/dev/ttyS1 pty,raw,echo=0,link=$HOME/dev/ttyS2 2>&1

