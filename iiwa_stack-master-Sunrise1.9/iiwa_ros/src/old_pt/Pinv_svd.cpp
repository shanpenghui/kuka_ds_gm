#include "stdafx.h"
#include<iostream> 
#include<Eigen/Core> 
#include<Eigen/SVD>   


template<typename _Matrix_Type_> 
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = 
    std::numeric_limits<double>::epsilon()) 
{  
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);  
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);  
    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint(); 
}   
int main() 
{  
    Eigen::MatrixXd A(2,3);  
    A<< 1, 2, 3, 4, 5, 7;  
    std::cout<<A<<std::endl<<std::endl;  
    std::cout<<pseudoInverse(A)<<std::endl;   
    Eigen::MatrixXd B(3,2);  
    B<< 1, 2, 3, 4, 5, 7;  
    std::cout<<B<<std::endl<<std::endl;  
    std::cout<<pseudoInverse(B)<<std::endl;  
    getchar();
    return 0; 
}