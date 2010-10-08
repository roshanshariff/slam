CPPFLAGS := -I. -I/opt/local/include/
CXXFLAGS := -Wall -O3

simulator: simulator.o
	g++ -o simulator simulator.o

simulator.o: simulator.cpp simulator.hpp slam/slam_data.hpp \
  utilities/arraymap.hpp slam/mcmc_slam.hpp slam/slam_data.hpp \
  utilities/random.hpp utilities/bitree.hpp utilities/random.hpp \
  utilities/bitree.hpp planar_robot/pose.hpp planar_robot/position.hpp \
  planar_robot/pose.hpp planar_robot/odometry_model.hpp \
  utilities/random.hpp utilities/geometry.hpp \
  planar_robot/range_bearing_model.hpp planar_robot/position.hpp \
  planar_robot/circle_controller.hpp utilities/bitree.hpp

clean:
	-rm simulator simulator.o
