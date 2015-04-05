#!/usr/bin/python
# monitor incoming buffers in flash disk, report out of sequence as error
# inotifywait -m -e close_write /mnt/disk/acq196_387/000000/*.id | ./test-scripts/check-buffers.py

import sys

prevnum = -1
nwraps = 0
errors = 0

while 1:
    next = sys.stdin.readline()         # read a one-line string
    if not next:                        # or an empty string at EOF
        break
    words = next.split()
    fullname = words[0]
    path = fullname.split('/')
    name = path.pop()
    number = int(name.split('.')[1])
   
    if prevnum < 0:
        prevnum = number
    elif number == prevnum+1:
        prevnum = number
	continue;
    elif number == 0:
        prevnum = number
	nwraps += 1
	print "wrap %d errors %d" % (nwraps, errors)
    else:
        print "number mismatch %d %d" % (prevnum, number)
        prevnum = number
        ++errors


