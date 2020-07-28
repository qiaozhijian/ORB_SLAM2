//
// Created by qzj on 2020/6/24.
//

#ifndef RENAMEIMAGE_UTIL_H
#define RENAMEIMAGE_UTIL_H

#include <iostream>
#include <stdint.h>
#include <string>
#include <stdio.h>
#include <fstream>

#include <boost/regex.hpp> //Note: using boost regex instead of C++11 regex as it isn't supported by the compiler until gcc 4.9



using namespace std;

void replace_str(std::string& str, const std::string& before, const std::string& after);
int32_t createDirectory(const std::string &directoryPath);
void copy_file(std::string src_, std::string dst_);
int sting2Int(string s);
bool exists_file(const std::string &name);
string getDirEnd(string dataset_dir);
string removeExtension(string filewhole);
void getSortedImages(const string img_path, vector<string> &img_paths);
bool DeleteFile(const char* path);
#endif //RENAMEIMAGE_UTIL_H
