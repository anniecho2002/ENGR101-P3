#include <iostream>
#include "E101.h"
#include <sys/time.h> // to check for dt

int v_goleft = 52;
int v_goright = 44;
int v_left, v_right;
bool run = true;
int quad = 1;

// proportional coefficient and derivative
double kp = 0.0005; 
double kd = 0.025;
double dv; // diff between speed L&R
double de;
double prevError = 0;
int row = 100;
// de is change in error, found diff between last error and last time

int redPixelCount;


struct timespec tStart; 
double dt;
using namespace std;




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
                return -160;
        } else {
                cout<<"error = "<<error/(double)total<<endl;
                return error/(double)total;
        }
}


void findDV(double error, double de){
        cout << "kp: " << kp << endl;
        dv = (error * kp); // + (kd * de);
        v_left = v_goleft + dv;
        v_right = v_goright + dv;
        cout<<"vleft = "<<v_left<<endl;
        cout<<"vright = "<<v_right<<endl;
        //cout<<"dv = "<<dv;
        changeMotors();
}



void wigglyLine(double error, timespec tEnd){
        long timePassed = 0; // in seconds (not nano-seconds)
        if (tStart.tv_nsec != tEnd.tv_nsec){
                timePassed = (tEnd.tv_sec - tStart.tv_sec)*1000000000 + tEnd.tv_nsec - tStart.tv_nsec;
                double timePassed2 = timePassed/1000000000;
                de = (error - prevError)/timePassed2;
                // de is the change in error over change in time
                tStart = tEnd;
                // resets the timer
        }
        findDV(error, de);
}

int main(){
        int err = init(0);
        cout<<"Hello!"<<err<<endl;
        open_screen_stream();
        openGate();

        while (run==true){
                if (quad==2){
                        take_picture();
                        double error = findError(100);
                        findDV(error, 0);
                        //struct timespec tEnd;
                        //clock_gettime(CLOCK_MONOTONIC, &tEnd);
                        //wigglyLine(error, tEnd);
                        prevError = error;
                }
        }
}

