#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <vector>
#include "iiwaRos.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64.h"
#include <fstream>
#include <string>
#include <stdio.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/Core> 
#include <Eigen/SVD>  
#include "math.h"  
#include "MathLib.h"
#include "GMR.h"
//#include "GMM.h"
//#include "MathLib.h"
//#include "Myadd.h"

using namespace std;
using namespace Eigen;
using namespace MathLib;

//float MyaaDD(float a, float b);

/*The SEDS model that you will use for motion generation.
This model should be accessible in your entire code. */

GaussianMixture mySEDS;

//////////调试链接用
////////GaussianMixturem mySEDS1;

////////int main()
////////{


////////	float a = 1;
////////	float b = 2.2;
////////	float c = 0;
////////	float d = 0;
////////	c = MyaaAA(a,b);
//////////	d = MyaaBB(a,b);
////////	printf("%f,%f",c,d);

int main()
{
//读取seds文件的起始和结尾
	bool b_SEDSLoaded = mySEDS.loadParams("WRSEDSModel1.txt");//WRSEDSModel3.txt
		if (b_SEDSLoaded)
		std::cout << "The SEDS Model is loaded successfully" << std::endl;
		else
		std::cout << "Error: Cannot find the SEDS model!!!" << std::endl;

//读取GMM文件的起始和结尾以及中间数组
	char filename1[]="WRGMMGMRModel2.txt";

		std::ifstream in(filename1);//, ios::trunc| ios::in | ios::out | ios::binary
		//string line;
		// int line;
	
		if(!in.is_open())
		{
			std::cout<<"GMM file open fail"<<std::endl;
			exit(1);
		}
//从txt传数据到数组向量
	std::vector<float> x_begin; std::vector<float> x_end; 
	std::array<float,3> x_begin_array; std::array<float,3> x_end_array;
	float x_begin_shuzu[3]; float x_end_shuzu[3];
	std::vector< std::vector<float> > x_position(3, std::vector<float>(0)); 
	float current_number = 0;
	for (int i = 3; i < 6; i++)
		{
			in >> current_number;
			x_begin.push_back(current_number);
			x_begin_array[i-3]=current_number;
			x_begin_shuzu[i-3]=current_number;
		}
	for (int j = 7; j < 10; j++)
		{
			in >> current_number;
			x_end.push_back(current_number);
			x_end_array[j-7]=current_number;
			x_end_shuzu[j-7]=current_number;
		}
	for (int k = 0; k < 3; k++)
		{
			for (int l = 0; l < 6100; l++)
			{
				in >> current_number;
				x_position[k].push_back(current_number);
			}
		}
	in.close();//关闭文件

	// // 验证输出结果
	// cout << "The x_begin are: ";
	// for (int count = 0; count < x_begin.size(); count++){
	// 	cout << x_begin[count] << " ";
	// 	cout << x_begin_array[count] << " ";
	// 	cout << x_begin_shuzu[count] << " ";
	// 	}
	// cout << "The x_end are: ";
	// for (int count = 0; count < x_end.size(); count++){
	// 	cout << x_end[count] << " ";
	// 	cout << x_end_array[count] << " ";
	// 	cout << x_end_shuzu[count] << " ";
	// 	}
	// cout << "The x_position are: ";
	// for (int m = 0; m < 3; m++)
	// {
	// 	for (int count = 0; count < 3; count++){//x_position[m].size()
	// 	cout << x_position[m][count] << " ";
	// 	}
	// }
	// cout << endl;


//运行SED模型
	int d=3;
	MathLib::Vector x,xd,xT; //defining the required variables
	x.Resize(d); //d is the dimensionality of  your model
	xd.Resize(d);
	/* Set the input value based on how you have defined yourSEDS model.
	For example, x could be the position ofthe robot's end-effector. */

//定义目标位置xT	
	for (int i = 0; i < 3; i++)
		{
			current_number=x_begin_shuzu[i];
			xT[i]=current_number;
		}
	//查看向量传递结果
	cout << "The xT are: ";
	for (int count = 0; count < 3; count++){
		cout << xT[count] << "; ";
		}

//定义当前位置x
	// // // // // // // for (int i = 0; i < 3; i++)
	// // // // // // // 	{
	// // // // // // // 		current_number= /*改为目前机器人位置*/[i];
	// // // // // // // 		x[i]=current_number;
	// // // // // // // 	}
	//查看向量传递结果
	cout << "The x are: ";
	for (int count = 0; count < 3; count++){
		cout << x[count] << "; ";
		}

//运行模型回归
	// x -= xT; //Transformation into the target frame of  reference
	// mySEDS.doRegression(x,xd);  // Estimating xd at x
	// xd.Print("xd = "); //Printing the value of  xd
//给机器人速度与阻抗命令


	return 0;

}
