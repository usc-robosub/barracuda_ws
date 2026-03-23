#!/usr/bin/env python3
"""Wrapper that exposes the package node as an executable for ROS2 launch."""
import sys
from ping360_node.node import main as ping360_main

if __name__ == "__main__":
    ping360_main(args=sys.argv[1:])