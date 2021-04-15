#include "fileOperation.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <chrono>
#include <ctime>

using namespace std;


string getTimeStr(){
    time_t now = chrono::system_clock::to_time_t(chrono::system_clock::now());

    string s(30, '\0');
    strftime(&s[0], s.size(), "%Y-%m-%d-%H-%M-%S", localtime(&now));
    return s;
}

void readCSV(string filename, vector<vector<double>>& outcome){
    // File pointer 
    fstream fin; 
  
    // Open an existing file 
    fin.open(filename, ios::in); 

    // Make sure the file is open
    if(!fin.is_open()){
        cout << "Could not open csv file: " << filename << endl;
        return;
    }

    // Helper vars
    string line, colname;
    double val; 
    // Read the column names
    if(fin.good())
    {
        // Extract the first line in the file
        getline(fin, line);

        // Create a stringstream from line
        stringstream ss(line);

        // Extract each column name
        while(getline(ss, colname, ',')){
            // Initialize and add <colname, int vector> pairs to result
            cout << colname << ", ";
        }
        cout<<endl;
        
        // Read data, line by line
        while(getline(fin, line))
        {
            // Create a stringstream of the current line
            ss = stringstream(line);
            
            // Keep track of the current column index
            int colIdx = 0;
            vector<double> lineArr;
            // Extract each data
            while(ss >> val){
                lineArr.push_back(val);
                // If the next token is a comma, ignore it and move on
                if(ss.peek() == ',') ss.ignore();
                // Increment the column index
                colIdx++;
            }
            outcome.push_back(lineArr);

        }
    }
    fin.close();
}

void writeCSV(std::fstream& outstream, uint64_t time_us, float* data, int size){
    string sp = ",";
    outstream << time_us << sp;
    for(int i = 0; i < size; i++){
        outstream << data[i] << sp;
    }
    outstream << endl;
}

void writeInfo(std::fstream& outstream, uint64_t time_us, std::string info){
    outstream << time_us << " -- " << info << endl; 
}