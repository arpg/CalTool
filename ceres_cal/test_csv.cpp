#include <CSVReader.h>

int main(int argc, char** argv)
{
  if (argc < 2)
    {
      cout << "Usage: test_csv <filename>" << endl;
      return -1;
    }
  
  CSVReader cam1(new string(argv[1]));

  vector<double> splits = cam1.getNextLineAsDouble();
  while (!splits.empty())
    {
      for (auto &theItem : splits)
	{
	  cout << theItem << ",";
	}
      cout << endl;
      splits = cam1.getNextLineAsDouble();
    }
  
}
