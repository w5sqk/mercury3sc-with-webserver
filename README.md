# mercury3sc-with-webserver

6/09/2023 w5sqk
v1.0 
Initial merge of webserver examples code by Khoi Hoang (noted below) and mercury3sc code

initial code working
ip address via dhcp 
webdisplay @ http://<ip address> port 80
displays same amplifier values as K9SUL's serial console 't' command, refreshes every 1 seconds, future effort to move to socket based 
  and push updates as amplifier updates Nextion dispay, may need to throttle updates, work in progress
  
Initial code summary: 
  renamed serial console print outputs to match mercury3sc  serial console prints.
  moved code supporting various arduino cpus to boardDefs module- code otherwise unchanged
  moved web initializations and print outputs to WebCheck. Added simple one to one print outputs to match serial 't" outputs.
  data varibles exceed nano 2k memery, moved to mega 2560 for code development
  
