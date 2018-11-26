#include "ros/ros.h"
#include "iiwaRos.h"
#include <iostream>  
#include <string>  
#include <vector>  
#include <fstream>  
using namespace std;  
  
/* 
@in, src: ���ָ���ַ��� 
@in, delim: �ָ����ַ��� 
@in_out, dest: ����ָ���ÿ���ַ��� 
*/  
void split(const string& src, const string& delim, vector<string>& dest)  
{  
    string str = src;  
    string::size_type start = 0, index;  
    string substr;  
  
    index = str.find_first_of(delim, start);    //��str�в���(��ʼ��start) delim�������ַ��ĵ�һ�γ��ֵ�λ��  
    while(index != string::npos)  
    {  
        substr = str.substr(start, index-start);  
        dest.push_back(substr);  
        start = str.find_first_not_of(delim, index);    //��str�в���(��ʼ��index) ��һ��������delim���ַ����ֵ�λ��  
        if(start == string::npos) return;  
  
        index = str.find_first_of(delim, start);  
    }  
}  
  
  
int main(int argc, char **argv) 
{   
ros::init(argc, argv, "read");
    ifstream infile("NDI.txt", ios::in);  
    vector<string> results;  
    string word;  
    string delim(" ");  
    string textline;  
    if(infile.good())  
    {  
        while(!infile.fail())  
        {  
            getline(infile, textline);  
            split(textline, delim, results);  
        }  
    }  
    infile.close();  
  
    vector<string>::iterator iter = results.begin();  
    while(iter != results.end())  
    {  
        cout<<*iter++<<endl;  
    }  
  
    return 0;  
}  
