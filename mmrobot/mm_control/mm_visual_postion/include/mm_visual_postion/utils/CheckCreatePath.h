#ifndef __CHECKCREATEPATH_H__
#define __CHECKCREATEPATH_H__
#include <iostream>
#include <string>
#include <sys/stat.h> // stat
#include <errno.h>    // errno, ENOENT, EEXIST
#if defined(_WIN32)
#include <direct.h>   // _mkdir
#endif

bool isDirExist(const std::string& path);

bool makePath(const std::string& path);


#endif 