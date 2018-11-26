#include "ros/ros.h"
#include "iiwaRos.h"
#include <iostream>  
#include <string>  
#include <vector>  
#include <fstream>  
using namespace std;  
  
/* 
@in, src: 待分割的字符串 
@in, delim: 分隔符字符串 
@in_out, dest: 保存分割后的每个字符串 
*/  
void split(const string& src, const string& delim, vector<string>& dest)  
{  
    string str = src;  
    string::size_type start = 0, index;  
    string substr;  
  
    index = str.find_first_of(delim, start);    //在str中查找(起始：start) delim的任意字符的第一次出现的位置  
    while(index != string::npos)  
    {  
        substr = str.substr(start, index-start);  
        dest.push_back(substr);  
        start = str.find_first_not_of(delim, index);    //在str中查找(起始：index) 第一个不属于delim的字符出现的位置  
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
