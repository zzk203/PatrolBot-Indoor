#!/bin/bash
ros2 service call /patrol/stop_patrol std_srvs/srv/Trigger "{}"
