#!/bin/bash

#Begin position
rostopic pub --once /SSC32U_request_topic std_msgs/String "data: '#0P1500#1P2500S500#2P500#3P1000#4P1500#5P1200T2000'" 
sleep 2s
#move to Cup
rostopic pub --once /SSC32U_request_topic std_msgs/String "data: '#0P1500#1P1150S500#2P500#3P900T2000'" 
sleep 2s
#grab
rostopic pub --once /SSC32U_request_topic std_msgs/String "data: '#5P1800T1000'" 
sleep 2s
#back to park with Cup
rostopic pub --once /SSC32U_request_topic std_msgs/String "data: '#0P1500#1P2500S500#2P500#3P1000T2000'" 
sleep 2s
#rotate
rostopic pub --once /SSC32U_request_topic std_msgs/String "data: '#0P500T2000'" 
sleep 2s
#drop pos
rostopic pub --once /SSC32U_request_topic std_msgs/String "data: '#1P1150S500#2P500#3P900T2000'" 
sleep 2s
#let it go
rostopic pub --once /SSC32U_request_topic std_msgs/String "data: '#5P1200T1000'"
sleep 2s
#back to park
rostopic pub --once /SSC32U_request_topic std_msgs/String "data: '#0P1500#1P2500S500#2P500#3P1000#4P1500#5P1200T2000'" 
sleep 5s
#Rotate back
rostopic pub --once /SSC32U_request_topic std_msgs/String "data: '#0P500T2000'" 
sleep 2s
#back to cup
rostopic pub --once /SSC32U_request_topic std_msgs/String "data: '#1P1150S500#2P500#3P900T2000'" 
sleep 2s
#grab
rostopic pub --once /SSC32U_request_topic std_msgs/String "data: '#5P1800T1000'"
sleep 2s
#rotate
rostopic pub --once /SSC32U_request_topic std_msgs/String "data: '#0P1500T2000'" 
sleep 2s
#let it go Encore Edition
rostopic pub --once /SSC32U_request_topic std_msgs/String "data: '#5P1200T1000'"
sleep 2s
#Begin position
rostopic pub --once /SSC32U_request_topic std_msgs/String "data: '#0P1500#1P2500S500#2P500#3P1000#4P1500#5P1200T2000'" 


