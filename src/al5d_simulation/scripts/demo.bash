#!/bin/bash

#Begin position
rostopic pub --once /SSC32U_request_topic std_msgs/String "data: '#0P1500#1P2500S500#2P500#3P1000#5P1200T2000'" 
sleep 2s
#move to Cup
rostopic pub --once /SSC32U_request_topic std_msgs/String "data: '#0P1500#1P1100S500#2P500#3P900T2000'" 
sleep 2s
#grab
rostopic pub --once /SSC32U_request_topic std_msgs/String "data: '#5P1750T1000'" 
sleep 2s
#back to park with Cup
rostopic pub --once /SSC32U_request_topic std_msgs/String "data: '#0P1500#1P2500S500#2P500#3P1000T2000'" 
sleep 2s
#rotate
rostopic pub --once /SSC32U_request_topic std_msgs/String "data: '#0P500T2000'" 
sleep 2s
#drop pos
rostopic pub --once /SSC32U_request_topic std_msgs/String "data: '#1P1120S500#2P500#3P900T2000'" 
sleep 2s
#let it go
rostopic pub --once /SSC32U_request_topic std_msgs/String "data: '#5P1200T1000'"
sleep 2s
#back up
rostopic pub --once /SSC32U_request_topic std_msgs/String "data: '#1P2500S500#2P500#3P1000T2000'" 
sleep 2s
#back to cup
rostopic pub --once /SSC32U_request_topic std_msgs/String "data: '#1P1120S500#2P500#3P900T2000'" 
sleep 2s
#grab
rostopic pub --once /SSC32U_request_topic std_msgs/String "data: '#5P1750T1000'"
sleep 2s
#rotate
rostopic pub --once /SSC32U_request_topic std_msgs/String "data: '#0P1500#1P2500S500#2P500#3P1000'" 
sleep 2s
#let it go Encore Edition
rostopic pub --once /SSC32U_request_topic std_msgs/String "data: '#5P1200T1000'"
sleep 2s
#Begin position
rostopic pub --once /SSC32U_request_topic std_msgs/String "data: '#0P1500#1P2500S500#2P500#3P1000#5P1200T2000'" 



