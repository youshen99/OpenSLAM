#ifndef FILEOPERATIONTOOL_H
#define FILEOPERATIONTOOL_H
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
namespace YOUSHEN_SLAM{
class FileOperationTool{
    public:
    void WriteFileByRowInFile(const std::string inputString, const std::string file_path, std::ios_base::openmode __mode);
    //template <typename T>
    void WriteFileByRowList(std::vector<double> &inputStringList, const std::string file_path, std::ios_base::openmode __mode);
    void ReadFileByFile(const std::string file_path, std::vector<std::string> &outputStringList);
};
}
#endif /* FILEOPERATIONTOOL_H */
