#!/usr/bin/env python

import sys

def main():
    fname = sys.argv[1]
    uav_name = sys.argv[2]
    path = sys.argv[3]
    
    with open(fname, 'r') as fhandle:
        for line in fhandle.readlines():
            line = line.replace("[REPLACEME]uav_name[/REPLACEME]", uav_name)
            line = line.replace("[REPLACEME]path[/REPLACEME]", path)
            print(line)

if __name__ == "__main__":
    main();
