#!/bin/bash

rosrun topic_tools relay /cmd_vel /rosbot/cmd_vel

# exit gracefully by returning a status 
exit 0
