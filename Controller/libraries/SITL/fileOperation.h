#include <vector>
#include <fstream>
#include <string>

void readCSV(std::string filename, std::vector<std::vector<double>>& outcome);
void writeCSV(std::fstream& outstream, uint64_t time_us, float* data, int size);
void writeInfo(std::fstream& outstream, uint64_t time_us, std::string info);
std::string getTimeStr();