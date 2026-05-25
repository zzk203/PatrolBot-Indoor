#!/bin/bash
ros2 service call /patrol/start_patrol std_srvs/srv/Trigger "{}"
