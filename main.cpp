// This code generate a trajectory from input command while make sure that it will not exceed the allowed acceleration and speed. This piece of code
// is prepaid to ultimately be implemented on an microprocessor; however, it must be modified to be implemented in a microprocessor

#include <iostream>
#include <math.h>
#include <cmath>
#include <fstream>
#include<string.h>
#include <stdlib.h>

using namespace std;


    double time = 10; //seconds
    double ts = 0.001; // 1 mili seconds

    double num_sample = time/ts;
    int Num_Sample = num_sample;



void command_input(int traj_nummber);

int main()
{
    /// Defining variables

    /// Dynamic variables
    double X[Num_Sample];
    double dX[Num_Sample];
    double dinput[Num_Sample];
    double Time[Num_Sample];
    double input[Num_Sample];


    command_input(2); //generate input command, write it it text file, input of function is number of input

    ifstream infile;
    int i=0;
    char cNum[10] ;
                infile.open ("command_input.txt", ifstream::in);
                if (infile.is_open())
                {

                        while (i<Num_Sample)
                        {
                                infile.getline(cNum, 256, ',');
                                input[i]= atof(cNum) ;
                                i++ ;
                                cout << input[i-1];
                        }
                        cout << "file is opened";
                        infile.close();
                }
                else
                {
                        cout << "Error opening file";
                }

    cout << input[2000] << "something something";


    //Read Trajectory file



    /// System limits
    // based on system requirements and limitation; these parameters is determined based on scanner limits and requirements
    double acc_max = 30;
    double vel_max = 35;
    double input_min=-20;
    double input_max = 20;

    ///System switching parameters
    // those parameters are determined based on trajectory nature and required accuracy; so they are not function of motor, instead function of application
    double Epsilon = 0.05;
    double epsilon = 0.005;
    double Epsilon1 = 0.05;
    double Epsilon2 = 0.1;
    double Delta = 0.05;
    double delta = 0.005;

    double alpha=0.01;
    double beta=0.01;

    /// Loop initialization
    double acc=0;
    double vel = 0;  // determine speed and velocity of target
    double X_setpoint=0;
    double dX_setpoint=0;
    int Direction;
    double D=0;
    double ACC_max=0;
    double VEL_max=0;
    double th1=0;
    double xh1=0;
    double th2=0;
    double xh2=0;
    double vel_p;
    double V1;
    double V2;
    double th1_p;
    double th2_p;
    double t1, t2, t3;
    double tL, xL;
    double Change_Time;
    double ddX;

    // Flags
    bool Change_Flag =0;

    bool Timer1_set =0;  //Timer flags
    bool Timer2_set =0;
    bool Timer3_set =0;

    bool IO_gap=0;            // Mode flags
    bool Major_input_change=0;
    bool Minor_input_change=0;
    bool First_order_regulator=0;
    bool Switch_to_input=0;
    bool set_point_change=0;
    bool Full_Load=0;
    bool Unvalid_trajectory=0;

    /// PI regulator initialization

    double vel_regulator = 0; // to correct small deviation and steady state error

    double D1 = 0 ;
    double vel_regulator_max= 0.5;
    double intD2 = 0;



/////////////////////////////////////////////////////////////////////////////////
////////////.............. main For Loop ..................//////////////////////
/// For loop

    for (int k = 1; k < Num_Sample ; k++) {
            // Time
            Time[k]=Time[k-1]+ts;


        dinput[k] = (input[k]-input[k-1])/ts;

        if (input[k] > input_max){
            input[k] = input_max;
            }
        else if (input[k] < input_min){
            input[k] = input_min;
        }

        /// Determine if there is a change

        //Full load working mode. when we already following a change
        if (!(Timer1_set || Timer2_set || Timer3_set )){
        Full_Load =true;
        }

        // input change, when input signal change abruptly
//        if (((input[k] - input[k-1])^2+alpha*(dinput[k]-dinput[k-1])^2) > Delta) // when it is to big, we consider it a change although we are in Full Load work
        double Dist1 = pow((input[k] - input[k-1]),2)+alpha*pow((dinput[k]-dinput[k-1]),2) ;
        if (Dist1 > Delta)
        {
            Major_input_change=1;
        }
        else if (Dist1 < Delta && Dist1 > delta && Full_Load==0) // For smaller change, we call them a change only if we are not in full load mode
        {
        Minor_input_change = 1;
        }

        // set point change, When we are in full load mode, input has changed considerably from our inital set point
        double Dist2 = pow((input[k] - X_setpoint),2)+beta*pow((dinput[k]-dX_setpoint),2) ;
        if (Full_Load==1 && Dist2 > Epsilon2)
            set_point_change=1;



        // (input- X) error , When there is an error between input and output
        double Dist3 = pow((input[k] - X[k-1]),2)+beta*pow((dinput[k]-dX[k-1]),2) ;
    if  (Full_Load==false) // Check if we are not in full load tracking, otherwise wait until that period is over
    {
        if (Dist3 > Epsilon){
        IO_gap = 1;}
        else if (Dist3 < Epsilon1 && Dist3 > epsilon){
            First_order_regulator = 1;} // To be completed, to remove Steady state errors in ramps
        else if (Dist3 < epsilon) {
        Switch_to_input = 1;} // To be completed later
    }


    //If any of those happens, we call it change and will ready to go to full load mode to compensate it
    if  (Major_input_change || Minor_input_change || set_point_change || IO_gap || Unvalid_trajectory)
    {
        Change_Flag = 1;
        Timer1_set =0; //reset all timers
        Timer2_set =0;
        Timer3_set =0;

   if ( abs(dinput[k])>10*vel_max ) // This is for when there is a jump in input, call change in next iteration to avoid big speed setpoint, it is not the best implementation, change it later,
      {
          Change_Flag =0;
        Unvalid_trajectory=1;}
    }

    /// %% PI regulator to eliminate steady state error

    if ( !(Change_Flag) && First_order_regulator)
        {

        D1 =  input[k] - X[k-1];
        intD2 = intD2+D1;
        vel_regulator = 0.1*(D1+0.005*intD2);
        // vel_regulator goes through a limiter
        if (vel_regulator > vel_regulator_max){
            vel_regulator = vel_regulator_max;
            }
        else if (vel_regulator < -vel_regulator_max){
            vel_regulator = -vel_regulator_max;
        }

    else
        {vel_regulator = 0;
        D1 = 0;
        intD2 = 0;
        }
    }
    /// Computing Trajectory parameters

    if (Change_Flag == 1) {
        D =  input[k] - X[k-1];

        X_setpoint= input[k]; // store those values to use in set_point_change identifier
        dX_setpoint = dinput[k];



        // determine the direction of first acceleration

        double First_acceleration_direction = D + beta*(dinput[k]-dX[k-1]);
        if (First_acceleration_direction>0)
        {
            Direction =1;
        }
        else {
            Direction =-1;
        }

        //computation parameters
        //Calculate the path based on second order second movement law

        ACC_max = Direction*acc_max;
        VEL_max = Direction*vel_max;
        th1 = (VEL_max-dX[k-1])/ACC_max; // time to reach maximum speed from zero speed with maximum acceleration at beginning of movement
        xh1 = 1/2*ACC_max*pow(th1,2)+dX[k-1]*th1; // distance that passed to reach maximum speed from zero speed with maximum acceleration at beginning of movement

        th2 = (VEL_max-dinput[k])/ACC_max; // time to reach maximum speed from zero speed with maximum acceleration at end of movement
        xh2 = -1/2*ACC_max*pow(th2,2)+VEL_max*th2; // distance that passed to reach maximum speed from zero speed with maximum acceleration at end of movement





        if (abs(D) < abs(xh1)+abs(xh2) )// shows we will not reach to maximum speed different in this case
        //first calculate vel_p
       {
       double a1 = 1/ACC_max;
       double c2 =  -1/(2*ACC_max)*(pow(dX[k-1],2)+pow(dinput[k],2))-D;
       double rr = abs(c2/a1);
       double r = pow(rr,0.5);
       double r1 = r;
       double r2 = -r;


       // choose a root that gives positive th1_p, and th2_p
       if ( (r1-dX[k-1])/ACC_max > 0 && (dinput[k]-r1)/-ACC_max> 0  )
       {
           vel_p=r1;
           Unvalid_trajectory=0;
       }
       else if ((r2-dX[k-1])/ACC_max > 0 && (dinput[k]-r2)/-ACC_max> 0 )
       {
           vel_p = r2;
           Unvalid_trajectory=0;
       }
       else
       {


           cout << "unvalid trajectory at " << k;
           Change_Flag=0;
           vel_p=0; // different from Matlab, take care of it
           Unvalid_trajectory=1;
       }
                                /*% we skip the change, that happends when
                                  % input speed change instanstly
                                  %and input position don't change much
                                  % strategy we cannot do anything, we just
                                  % wait till another change identificator
                                  % catch the error
                                  % however, we must modiy this part later

                                  % sometimes it gives some answer but not
                                  % a good one, we must understand when
                                  % that happens exacly and do something
                                  % else for those case, the answer it that
                                  % we apply d direction based on D, but
                                  % when D is really small we must to that
                                  % based on V ==> so use norm!!!
                                  */


       V1= dX[k-1];
       V2 = dinput[k];

       th1_p = (vel_p-dX[k-1])/ACC_max;
       //xh1_p = 1/2*ACC_max*th1_p^2+dX(k-1)*th1_p; % just for checking

        th2_p = (dinput[k]-vel_p)/(-ACC_max);
        //xh2_p = 1/2*ACC_max*th2_p^2+dinput(k)*th2_p; % just for checking
       //D_p=xh1_p + xh2_p; % just for checking

       t1 = th1_p;
       t2  =0;
       t3 = th2_p;
       }


        if (abs(D) > abs(xh1)+abs(xh2))
        {
            xL = D-xh1-xh2; // distance that we go with maximum speed;
            tL =  xL/VEL_max; // time that we travel with maximum speed;
            Unvalid_trajectory=0;
            t1 = th1;
            t2= tL;
            t3 = th2;
        }



        if (Unvalid_trajectory==0) //It trajecotry is unvalid, skip it, we need to add alternative strategy for those cases later
            {Timer1_set =1;
            }

        Change_Time=Time[k];  // save the time for using in timer, actually we using our timer here
        Change_Flag = 0;      // reset all flags
        IO_gap = 0;
        Major_input_change=0;
        Minor_input_change=0;
        First_order_regulator=0;
        Switch_to_input=0;
        set_point_change=0;
        vel_regulator = 0;

    } // end of if change


        /// First step of movement, accelerate
        if (Timer1_set)
        {
            acc = ACC_max;
            vel = 0;
            // check the timer1 timeout
            if (Change_Time + t1 < Time[k]){
                Timer1_set =0;
                Timer2_set = 1;
                Change_Time = Time[k];
            }
        }

        /// Second step of movement, moving with maximum speed
        if (Timer2_set){
            acc = 0;
            vel = VEL_max;

        // check the timer2 timeout
            if (Change_Time + t2 <= Time[k])
            {
            Timer2_set =0;
            Timer3_set = 1;
            Change_Time = Time[k];
            }
        }


        /// Thired step of movement, decelerate
        if (Timer3_set)
            {
            acc = -ACC_max;
            vel = 0;
            // check the timer3
            if (Change_Time +t3 < Time[k])
                {
                Timer3_set = 0;
                Full_Load = 0;
                acc=0;
                vel = dinput[k]; // change it
                }
            }




        /// calculating trajectory, based on
        ddX= acc;
        if (vel ==0)  // Choose between linear system and quadratic system
            {
            dX[k]=dX[k-1]+ts*ddX;
            }
        else
            {
            dX[k]=vel+vel_regulator;
            }
        X[k]=X[k-1]+ts*dX[k];


        /// Add switch part later here


} // simulation loop is finished








    /// Plotting

    //saving filtered input in txt file
        ofstream myfile ("filtered output.txt");
        if (myfile.is_open())
        {
    //myfile << "This is a line.\n";
    //myfile << "This is another line.\n";
        for(int count = 0; count < Num_Sample; count ++){
            myfile << X[count] << "," ;
        //myfile << count << "," ;
        }
        myfile.close();
        }
        else cout << "Unable to open file";



        //plotting


    return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions

void command_input(int traj_nummber){

////// Define input_command number 1 ////////////////
    double arr1[Num_Sample];
    double *p1 =arr1;

//        for (int i = 0; i < Num_Sample ; i++) {
//        arr1[i] = i;
//}


    for (int i = 1000; i < 3000 ; i++) {
        arr1[i] = 1.1;
}

    for (int i = 3000; i < 5000 ; i++) {
        arr1[i] = -1;
}

    for (int i = 7000; i < 10000 ; i++) {
        arr1[i] = arr1[i-1]+1*ts;
}

/////////  Define input_command number 2 //////////////////////////////////////

    double arr2[Num_Sample];
    double *p2 =arr2;


    for (int i = 1000; i < 2000 ; i++) {
        arr2[i] = arr2[i-1]+1*ts;
}

    for (int i = 2000; i < 4000 ; i++) {
        arr2[i] = arr2[i-1]-1.5*ts;
}

    for (int i = 5000; i < 6000 ; i++) {
        arr2[i] = 2;
}

    for (int i = 7000; i < 10000 ; i++) {
        arr2[i] = arr2[i-1]+-1*ts;
}

////////////////////////////////////////////////////////////////////////

    // select between command inputs ///////////////////
    double* p_selected;
    switch(traj_nummber) {
   case 1  :
      cout << "First trajectory is chosen "<< endl;
      p_selected = p1;
      break;
   case 2  :
      cout << "Second  trajectory is chosen" << endl;
      p_selected = p2;
      break;
    default :
         cout << "Invalid Trajectory number " << traj_nummber << endl;
      break;
        }

/////////// Write in a file //////////
    ofstream myfile ("command_input.txt");
      if (myfile.is_open())
  {

    for(int count = 0; count < Num_Sample; count ++){
        //myfile << arr1[count] << "," ;
        myfile << *(p_selected+count) << "," ;
    }
    myfile.close();
  }

  else cout << "Unable to open file";

}
