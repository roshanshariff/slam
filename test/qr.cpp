#include <iostream>
#include <iomanip>

#include <Eigen/Core>
#include <Eigen/Jacobi>
#include <Eigen/Cholesky>
#include <Eigen/QR>

int main () {

	std::cout << std::fixed << std::setprecision(4);

	Eigen::Matrix3d M;
	M << 8, 1, 6,
		 3, 5, 7,
		 4, 9, 2;
	M = M.transpose() * M;
	std::cout << "M = \n" << M << std::endl;

	Eigen::Matrix3d R = M.householderQr().matrixQR().triangularView<Eigen::Upper>();
	std::cout << "R = \n" << R << std::endl;

}
