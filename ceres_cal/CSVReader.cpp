#include <CSVReader.h>

CSVReader::CSVReader(string *m_srcFile, char delim)
{
   if( m_srcFile->empty() ) {
     cerr << __FILE__ << ": Unable to find file '" << m_srcFile->c_str() << "'" << endl;
    }
   else {
      srcFile.open( m_srcFile->c_str() );
      if( srcFile.is_open() == false ) {
	cerr << __FILE__ << ": Unable to open file '" << m_srcFile->c_str() << "'" << endl;
      }
    }

   
}

CSVReader::~CSVReader()
{
  if (srcFile.is_open())
    srcFile.close();
}

vector<string> CSVReader::getNextLineAsString()
{
  string sValue;
  getline(srcFile, sValue );
  vector<string> items = hal::Split(sValue, ',');
  return items;
}

vector<double> CSVReader::getNextLineAsDouble()
{
  string sValue;
  getline(srcFile, sValue );
  vector<string> items = hal::Split(sValue, ',');
  vector<double> items_d;
  
  for (auto &theItem : items)
    {
      items_d.push_back(atof(theItem.c_str()));
    }
  
  return items_d;
}
