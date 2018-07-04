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

  for (float initial_altitude = 000001; initial_altitude < 100000 ; initial_altitude++){


    float renewal_altitude = initial_altitude + rand() % 9 - 3;
    float delta_altitude = renewal_altitude - initial_altitude;
    bool motor_trigger = true;

    if(delta_altitude >= 000001 && motor_trigger == true  ){
      filew << initial_altitude << "\t" << renewal_altitude << " \t "<< delta_altitude << "\t " <<" : Flag : Opened" <<endl;
    }else
    {
      filew << initial_altitude << "\t" << renewal_altitude << " \t "<< delta_altitude << "\t " <<" : Flag : Closed" <<endl;
    }



  };
  filew.close();

}
