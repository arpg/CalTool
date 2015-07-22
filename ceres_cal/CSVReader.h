#ifndef __CSVREADER_H_
#define __CSVREADER_H_

#include <iostream>
#include <fstream>
#include <string>

#include <HAL/Utils/StringUtils.h>

using namespace std;

class CSVReader
{
 public:
  CSVReader(string *m_srcFile, char delim=',');
  ~CSVReader();
  vector<string> getNextLineAsString();
  vector<double> getNextLineAsDouble();
 private:
  ifstream srcFile;
  
};
#endif
