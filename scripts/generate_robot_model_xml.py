#!/usr/bin/env python3

import sys
import random

def color_to_text(color):
    return "{:.0e} {:.0e} {:.0e} {:.0e}".format(color[0], color[1], color[2], color[3])

def get_uav_color(uav_name):
    transp = 0.2
    colors = dict(
        uav1=(1.0, 0.0, 0.0, transp),
        uav2=(0.0, 1.0, 0.0, transp),
        uav3=(0.0, 0.0, 1.0, transp),
#
        uav4=(1.0, 1.0, 0.0, transp),
        uav5=(1.0, 0.0, 1.0, transp),
        uav6=(0.0, 1.0, 1.0, transp),
#
        uav7=(1.0, 0.5, 0.5, transp),
        uav8=(0.5, 1.0, 0.5, transp),
        uav9=(0.5, 0.5, 1.0, transp),

        uav=(0.6, 0.6, 0.6, transp)
    )
    if uav_name in colors:
        return color_to_text(colors[uav_name])
    else:
        return color_to_text(random.choice(list(colors.values())))

def main():
    fname = sys.argv[1]
    uav_name = sys.argv[2]
    path = sys.argv[3]

    uav_color = get_uav_color(uav_name)

    with open(fname, 'r') as fhandle:
        for line in fhandle.readlines():
            line = line.replace("[REPLACEME]uav_color[/REPLACEME]", uav_color)
            line = line.replace("[REPLACEME]uav_name[/REPLACEME]", uav_name)
            line = line.replace("[REPLACEME]path[/REPLACEME]", path)
            print(line)

if __name__ == "__main__":
    main();
