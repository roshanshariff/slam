#include <iostream>
#include <iomanip>

#include <Eigen/Core>
#include <Eigen/Cholesky>

#include "utility/cholesky.hpp"


int main () {

	std::cout << std::fixed << std::setprecision(4);

	Eigen::Matrix3d M;
	M << 8, 1, 6,
		 3, 5, 7,
		 4, 9, 2;
	M = M.transpose() * M;
	std::cout << "M = \n" << M << std::endl;

	Eigen::Vector3d v;
	v << 1, 2, 3;
	std::cout << "v = \n" << v << std::endl;

	Eigen::Matrix3d L = M.llt().matrixL();
	std::cout << "L = \n" << L << std::endl;
	std::cout << "L*L' = \n" << (L * L.transpose()) << std::endl;

	cholesky_update(L, v);
	std::cout << "Lnew = \n" << L << std::endl;
	std::cout << "Lnew*Lnew' = \n" << (L * L.transpose()) << std::endl;

	std::cout << "Mnew = \n" << (M + v*v.transpose()) << std::endl;

}
