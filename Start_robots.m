clear all

% script to start robots, Run once all the simulinks are switched on
robot0 =  tcpserver("192.168.0.5", 18012);
robot1 =  tcpserver("192.168.0.5", 18013);
robot2 =  tcpserver("192.168.0.5", 18014);
% wait to run the next section for robot initialization
 
% write(robot0,1,"double");
% write(robot1,1,"double");
% write(robot2,1,"double");