#include <iostream>
#include "E101.h"
#include <sys/time.h> // to check for dt

int v_goleft = 54;
int v_goright = 42;
int v_left, v_right;
bool run = true;
int quad = 1;

// proportional coefficient and derivative
double kp = 0.15; 
double kd = 0;
double dv; // diff between speed L&R
double de;

double prevError = 0;
int row = 120;
// row used to be 100
// de is change in error, found diff between last error and last time

int redPixelCount;
int intersectCount = 0;
int uTurn = 0;

using namespace std;




// quadrant one code
void openGate1(){
        char buffer[1000];
        char ip[] = "130.195.6.196";
        char please[] = "please";
        connect_to_server(ip, 1024);
        send_to_server(please);
        receive_from_server(buffer);
        send_to_server(buffer);

        quad = 2;
}

void openGate(){
        char server_addr[15] = "130.195.6.196";
        char message[24] = "Please";
        connect_to_server(server_addr, 1024);
        send_to_server(message);
        receive_from_server(message);
        send_to_server(message);

        quad = 2;
}


// function that changes the motor speed
void changeMotors(){
        set_motors(1, v_right);
        set_motors(5, v_left);
        hardware_exchange();
        //if (uTurn >= 75){
        //      sleep1(4000);
        //}
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
        if (redPixelCount > 100){
                uTurn = 0;
        }
        redPixelCount = 0;


        for (int i=0; i<320; i++){
                error = error + black[i]*middle[i];
                if (black[i] == 1) {
                        total++;
                }
        }

        if (total == 0) {
                //if there's no black pixels, return maximum turning
                // used to be -160 
                uTurn++;
                return -160;
        } else {
                cout<<"error = "<<error/(double)total<<endl;
                return error/(double)total;
        }
}


void wigglyLine(double error){
        cout << "kp: " << kp << endl;
        dv = (error * kp);
        v_left = v_goleft + dv;
        v_right = v_goright + dv;
        cout<<"vleft = "<<v_left<<endl;
        cout<<"vright = "<<v_right<<endl;
        cout<<"uturn = "<<uTurn<<endl;
        if (uTurn >= 75){
                v_left = v_goleft;
                v_right = v_goright;
        }
        changeMotors();
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
                        wigglyLine(error);
                }
        }
}
