CPPFLAGS := -I. -I/opt/local/include/
CXXFLAGS := -Wall -O3

simulator: simulator.o
	g++ -o simulator simulator.o

simulator.o: simulator.cpp planar_robot/pose.hpp utilities/random.hpp \
  utilities/geometry.hpp planar_robot/observation.hpp \
  planar_robot/pose.hpp planar_robot/odometry_model.hpp \
  slam/mcmc_slam.hpp slam/slam_data.hpp utilities/arraymap.hpp \
  utilities/random.hpp utilities/bitree.hpp slam/slam_data.hpp \
  utilities/random.hpp

clean:
	-rm simulator simulator.o
