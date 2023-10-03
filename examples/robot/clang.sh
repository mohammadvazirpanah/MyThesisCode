clang++ -I. -I/home/mohammad/Ros_Project/MyCode/ForSyDe-SystemC/src -I/opt/ros/noetic/include -L/opt/ros/noetic/lib -Wl,-rpath=/opt/ros/noetic/lib main.cpp -o main -lroscpp -lrosconsole -lrostime -lroscpp_serialization -lsystemc -lm 
#-DFORSYDE_INTROSPECTION



clang++ -I. -I/media/mohammad/E/Course/Thesis/Code-Backup/ROS_Wrapper/ForSyDe-SystemC/src -I/home/mohammad/systemc-2.3.3/include -I/opt/ros/noetic/include -L/home/mohammad/systemc-2.3.3/lib-linux64 -L/opt/ros/noetic/lib -Wl,-rpath=/opt/ros/noetic/lib main.cpp -o main -lroscpp -lrosconsole -lrostime -lroscpp_serialization -lsystemc -lm 



clang++ -I. -I/home/mohammad/ForSyDe-SystemC/src -I/home/mohammad/systemc-2.3.3/include -I/opt/ros/noetic/include -L/home/mohammad/systemc-2.3.3/lib-linux64 -L/opt/ros/noetic/lib -Wl,-rpath=/opt/ros/noetic/lib main.cpp -o main -lroscpp -lrosconsole -lrostime -lroscpp_serialization -lsystemc -lm 
