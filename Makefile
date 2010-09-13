geometry.o: geometry.cpp geometry.hpp

pose.o: pose.cpp pose.hpp geometry.hpp

probability.o: probability.cpp probability.hpp

odometry.o: odometry.cpp odometry.hpp geometry.hpp pose.hpp probability.hpp
