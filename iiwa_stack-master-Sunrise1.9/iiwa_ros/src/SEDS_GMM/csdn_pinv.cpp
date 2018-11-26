//#include "funset.hpp"
#include <math.h>
#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
//#include "common.hpp"
 
int test_pseudoinverse()
{
	//std::vector<std::vector<float>> vec{ { 0.68f, 0.597f },
	//				{ -0.211f, 0.823f },
	//				{ 0.566f, -0.605f } };
	//const int rows{ 3 }, cols{ 2 };
 
	std::vector<std::vector<float>> vec{ { 0.68f, 0.597f, -0.211f },
					{ 0.823f, 0.566f, -0.605f } };
	const int rows{ 2 }, cols{ 3 };
 
	std::vector<float> vec_;
	for (int i = 0; i < rows; ++i) {
		vec_.insert(vec_.begin() + i * cols, vec[i].begin(), vec[i].end());
	}
	Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> m(vec_.data(), rows, cols);
 
	fprintf(stderr, "source matrix:\n");
	std::cout << m << std::endl;
 
	fprintf(stderr, "\nEigen implement pseudoinverse:\n");
	auto svd = m.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
 
	const auto &singularValues = svd.singularValues();
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> singularValuesInv(m.cols(), m.rows());
	singularValuesInv.setZero();
	double  pinvtoler = 1.e-6; // choose your tolerance wisely
	for (unsigned int i = 0; i < singularValues.size(); ++i) {
		if (singularValues(i) > pinvtoler)
			singularValuesInv(i, i) = 1.0f / singularValues(i);
		else
			singularValuesInv(i, i) = 0.f;
	}
 
	Eigen::MatrixXf pinvmat = svd.matrixV() * singularValuesInv * svd.matrixU().transpose();
	std::cout << pinvmat << std::endl;
 
	return 0;
}
