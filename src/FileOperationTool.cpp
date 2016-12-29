#include "FileOperationTool.h"
namespace YOUSHEN_SLAM{

//仅仅写入一行数据
void FileOperationTool::WriteFileByRowInFile(const std::string inputString, const std::string file_path, std::ios_base::openmode __mode)
{

    std::ofstream in;
    in.open(file_path.data(), __mode);
    in << inputString << "\n";
    in.close();

};


void FileOperationTool::WriteFileByRowList(std::vector<double> &inputStringList, const std::string file_path, std::ios_base::openmode __mode)
{

    std::ofstream in;
    in.open(file_path.data(), __mode);
    for (size_t i = 0; i < inputStringList.size(); i++)
    {
        in << inputStringList[i] << "\n";
    }
    in.close();
   
};

void FileOperationTool::ReadFileByFile(const std::string file_path, std::vector<std::string> &outputStringList)
{

    std::ifstream fin(file_path.data());
    //  const int LINE_LENGTH = 1024;
    std::string s;
    while (std::getline(fin, s))
    {
        outputStringList.push_back(s);
    }

};
}
