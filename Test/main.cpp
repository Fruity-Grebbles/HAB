#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string>
using namespace std;

void writeFile();
void readFile();

int main(){

  writeFile();
  readFile();

  return 0;
}


void readFile(){
  string lines;

  ifstream filer("./dataset.txt");

  if(filer.is_open()){

  while (getline(filer, lines)){
     cout << lines << "\n";
  }
    filer.close();
  }
  else{
    cout << "unable to open the file";
  }
}

void writeFile(){

  ofstream filew;
  filew.open("dataset.txt");

  // This small simulation checked and written with Prof. Paul Griffo

  for (int initial_altitude = 0; initial_altitude < 1000 ; initial_altitude++){

    int target_altitude = 600;
    int altidude_second = initial_altitude + 1;
    int delta_altitude = altidude_second - initial_altitude;

    if(delta_altitude > 0 &&  initial_altitude > target_altitude ){
      int initial_altitude  = altidude_second ;
      filew << initial_altitude << "\t" << altidude_second << " \t "<< delta_altitude << "\t " <<" : Flag : Opened" <<endl;
    }else
    {

      filew << initial_altitude << "\t" << altidude_second << " \t "<< delta_altitude << "\t " <<" : Flag : Closed" <<endl;
    }

  };

for(int initial_altitude = 1000; initial_altitude > 0 ; initial_altitude--){

  int target_altitude = 600;
  int altidude_second = initial_altitude + 1;
  int delta_altitude =  initial_altitude - altidude_second;

  if(delta_altitude > 0 &&  initial_altitude > target_altitude ){
    filew << initial_altitude << "\t" << altidude_second << " \t "<< delta_altitude << "\t " <<" : Flag : Opened" <<endl;
  }else
  {

    filew << initial_altitude << "\t" << altidude_second << " \t "<< delta_altitude << "\t " <<" : Flag : Closed" <<endl;
  }
};



  filew.close();

}
