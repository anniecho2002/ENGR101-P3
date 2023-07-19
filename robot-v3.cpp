#include <iostream>
#include "E101.h"
#include <sys/time.h> // to check for dt
#include <vector> //  for vectors
#include <numeric>
#include <algorithm>

using namespace std;

int v_goleft = 52;
int v_goright = 44;
int v_left, v_right;
bool run = true;
int quad = 1;

// proportional coefficient and derivative
double kp = 0.025; 
double kd = 0;
double dv; // diff between speed L&R
double de;
double prevError = 0;
int row = 120;
// row used to be 100
// de is change in error, found diff between last error and last time

int redPixelCount;
// public scope varibles for completion
vector <int> lft;
vector <int> rght;
vector <int> top;
vector <int> scan;
vector <int> middleScan;
vector <int> scan2;
//double mazeError = 0;
//int turn = 0;
int bothBlack = 0;


struct timespec tStart; 
double dt;





// quadrant one code
void openGate(){
        char buffer[1000];
        char ip[] = "130.195.6.196";
        char please[] = "please";
        connect_to_server(ip, 1024);
        send_to_server(please);
        receive_from_server(buffer);
        send_to_server(buffer);

        quad = 2;
}


// function that changes the motor speed
void changeMotors(){
        set_motors(1, v_right);
        set_motors(5, v_left);
        hardware_exchange();
}


void stopMotors(){
        set_motors(1, 48);
        set_motors(5, 48);
        hardware_exchange();
}

void moveCamera(int move){
        set_motors(3, move);
        hardware_exchange();
}


// finds the error and returns value
double findError(double threshold){
        int black[320] = {0}; // checks along one entire row (all the cols) for the pixels
        int middle[320] = {0};
        double error = 0;
        int total = 0;

        // fills up the middle array so it can be multiplied
        for (int fill=0; fill<320; fill++){
                middle[fill] = fill - 160;
        }

        // checks along the entire row and finds the values
        for (int col=0; col<320; col++){
                int red = get_pixel(row,col,0);
                int green = get_pixel(row,col,1);
                int blue = get_pixel(row,col,2);
                int intensity = get_pixel(row,col,3);

                if(intensity > threshold){
                        black[col] = 0;
                        // there is a white pixel
                }
                else{
                        black[col] = 1;
                        // there is a black pixel
                }

                // for finding red line to change between quadrants
                if (red>(1.4*green) && red>(1.4*blue)){
                        redPixelCount++;
                }
        }
        for (int i=0; i<320; i++){
                error = error + black[i]*middle[i];
                if (black[i] == 1) {
                        total++;
                }
        }
        //cout<<"errorxkp = " <<error*kp<<endl;
        if (total == 0) {
                //if there's no black pixels, return maximum turning
                // used to be -160 
                return 160;
        } else {
                cout<<"error = "<<error/(double)total<<endl;
                return error/(double)total;
        }
}
// finds the error for completion part
double mazeLine(double threshold){
    bool blackLeft = false;
    bool blackRight = false;
    bool blackTop = false;
    bool turnLeft = false;
    bool turnLeft2 = false;
    
    //int turn = 0;
    //int bothBlack = 0;
    int total = 0;
    double mazeError = 0;
    scan.clear();
    lft.clear();
    rght.clear();
    top.clear();
    middleScan.clear();
    for(int leftRow = 235; leftRow > 5; leftRow--){
        int luminosity = get_pixel(leftRow, 5, 3);
        
        if(luminosity > threshold){
                        scan.push_back(0);
                        lft.push_back(0);
                        top.push_back(0);
                        rght.push_back(0);

                        // there is a white pixel
                }
                else{
                        scan.push_back(1);
                        lft.push_back(1);
                        blackLeft = true;
                        rght.push_back(0); // because this vector is for left side
                        top.push_back(0);
                        total++;
                        // there is a black pixel
                }
        
    }
    for (int topCol = 5; topCol < 315; topCol++ ){
        int luminosity = get_pixel(5, topCol, 3);
        if(luminosity > threshold){
                        scan.push_back(0);
                        top.push_back(0);
                        rght.push_back(0);
                        lft.push_back(0);
                        // there is a white pixel
                }
                else{
                        scan.push_back(1);
                        top.push_back(1);
                        blackTop = true;
                        rght.push_back(0);
                        lft.push_back(0); // because this vector is for top side
                        total++;
                        // there is a black pixel
                }
    }
    for( int rightRow = 5; rightRow < 235; rightRow++){
        int luminosity = get_pixel(rightRow, 315, 3);
        if(luminosity > threshold){
                        scan.push_back(0);
                        rght.push_back(0);
                        lft.push_back(0);
                        top.push_back(0);

                        // there is a white pixel
                }
                else{
                        scan.push_back(1);
                        rght.push_back(1);
                        blackRight = true;
                        lft.push_back(0);
                        top.push_back(0); // because this vector is for right side
                        total++;
                        // there is a black pixel
                }
    }
    for( int fill = 0; fill < scan.size(); fill++){
    middleScan.push_back(fill - (scan.size()/2.0));

    }
    
    if (blackLeft == true && blackRight == true){
     turnLeft = true;
     bothBlack++;
    }
    else if (blackLeft == true && blackRight == true && blackTop == true){
     turnLeft2 = true;
    }
    else{
            for(int i = 0; i < scan.size(); i++){
                    mazeError = mazeError + (scan.at(i)*middleScan.at(i));
            }
            mazeError = mazeError / (double) total;
    }
    if (turnLeft == true){
         scan2.clear();
    mazeError = 0;
    int totalLeft = 0;
    for(int i = 0; i < scan.size(); i++){
            if(lft.at(i) == 1){
                    scan2.push_back(1);
                    totalLeft++;
            }
            else{
                    scan2.push_back(0);
            }
    }
    for (int i = 0; i<scan2.size(); i++){
            mazeError = mazeError + (scan2.at(i)*middleScan.at(i));
    }
    mazeError = mazeError / (double) totalLeft;
    turnLeft = false;
      
    }
    if (turnLeft2 == true){
        scan2.clear();
    mazeError = 0;
    int totalLeft = 0;
    for(int i = 0; i < scan.size(); i++){
            if(lft.at(i) == 1){
                    scan2.push_back(1);
                    totalLeft++;
            }
            else{
                    scan2.push_back(0);
            }
    }
    for (int i = 0; i<scan2.size(); i++){
            mazeError = mazeError + (scan2.at(i)*middleScan.at(i));
    }
    mazeError = mazeError / (double) totalLeft;
    turnLeft2 = false;
    
       
    }
    if (bothBlack == 2){
          scan2.clear();
    mazeError = 0;
    int totalRight = 0;
    for( int i = 0; i<scan.size(); i++){
            if(rght.at(i) == 1){
                    scan2.push_back(1);
                    totalRight++;
            }
            else{
                    scan2.push_back(0);
            }
    }
    for (int i = 0; i<scan2.size(); i++){
            mazeError = mazeError + (scan2.at(i)*middleScan.at(i));
    }
    mazeError = mazeError / (double) totalRight;
    bothBlack = 10;
    
    }
    
    
    
 return mazeError;


    




}


void wigglyLine(double error, double de){
        cout << "kp: " << kp << endl;
        dv = (error * kp);// + (kd * de);
        v_left = v_goleft + dv;
        v_right = v_goright + dv;
        cout<<"vleft = "<<v_left<<endl;
        cout<<"vright = "<<v_right<<endl;
        //cout<<"dv = "<<dv;
        changeMotors();
}





int main(){
        int err = init(0);
        cout<<"Hello!"<<err<<endl;
        open_screen_stream();
        openGate();
        //moveCamera(60);

        while (run==true){
                if (quad==2){
                        take_picture();
                        double error = findError(100);
                        wigglyLine(error, 0);
                        if (redPixelCount> 200){
                                quad = 3;
                        }
                        //struct timespec tEnd;
                        //clock_gettime(CLOCK_MONOTONIC, &tEnd);
                        //wigglyLine(error, tEnd);
                        //prevError = error;
                }
                if (quad == 3){
                        take_picture();
                        double error = mazeLine(100);
                        wigglyLine(error, 0);
                }
        }
}
