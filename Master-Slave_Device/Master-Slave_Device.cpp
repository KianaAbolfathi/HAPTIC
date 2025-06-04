//include all libraries

#include <chrono>
#include <thread>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "dhdc.h"
#include <cmath>
#include <time.h>
#include <string>

#define REFRESH_INTERVAL  0.01   // sec

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}

inline void
MatTranspose(const double a[3][3], double m[3][3])
{
    m[0][0] = a[0][0];  m[0][1] = a[1][0];  m[0][2] = a[2][0];
    m[1][0] = a[0][1];  m[1][1] = a[1][1];  m[1][2] = a[2][1];
    m[2][0] = a[0][2];  m[2][1] = a[1][2];  m[2][2] = a[2][2];
}

int
main(int argc, char** argv)
{
    // define the force constants for each axes
    double Kx = 10;
    double Ky = 10;
    double Kz = 10;

    //define the positons of the haptic device
    double px, py, pz;

    //define the forces of the haptic device
    double fx, fy, fz;
    double j0, j1, j2;
    double g0, g1, g2;
    double q0, q1, q2;
    double J[3][3];
    double Jt[3][3];
    double freq = 0.0;
    double t1, t0 = dhdGetTime();
    int    done = 0;
    int    sat;
    int    initflag = 1;
    int    i = 0;

    using namespace std::this_thread; // sleep_for, sleep_until
    using namespace std::chrono; // nanoseconds, system_clock, seconds

    // required to access joint angles
    dhdEnableExpertMode();

    // open the first available device
    if (dhdOpen() < 0) {
        dhdSleep(2.0);
        return -1;
    }


    // emulate button on supported devices
    dhdEmulateButton(DHD_ON);

    // enable force
    dhdEnableForce(DHD_ON);

    // haptic loop
    while (!done) {

        // retrieve joint angles
        if (dhdGetPosition(&px, &py, &pz) < DHD_NO_ERROR) {
            done = 1;
        }

        // Apply forces
        fx = -Kx * px;
        fy = -Ky * py;
        fz = -Kz * pz;

        // retrieve joint angles
        if (dhdGetDeltaJointAngles(&j0, &j1, &j2) < DHD_NO_ERROR) {
            done = 1;
        }

        // compute jacobian
        if (dhdDeltaJointAnglesToJacobian(j0, j1, j2, J) < DHD_NO_ERROR) {
            done = 1;
        }

        // compute joint torques required for gravity compensation
        if (dhdDeltaGravityJointTorques(j0, j1, j2, &g0, &g1, &g2) < DHD_NO_ERROR) {
            done = 1;
        }

        // compute joint torques Q = ((J)T) * F
        MatTranspose(J, Jt);
        q0 = Jt[0][0] * fx + Jt[0][1] * fy + Jt[0][2] * fz;
        q1 = Jt[1][0] * fx + Jt[1][1] * fy + Jt[1][2] * fz;
        q2 = Jt[2][0] * fx + Jt[2][1] * fy + Jt[2][2] * fz;

        // combine gravity compensation and requested force
        q0 += g0;
        q1 += g1;
        q2 += g2;

        // apply joint torques
        if ((sat = dhdSetDeltaJointTorques(q0, q1, q2)) < DHD_NO_ERROR) {
            done = 1;
        }
        double pipex, pipey, pipez, pipeb;
        // display refresh rate and position at 10Hz
        t1 = dhdGetTime();
        if ((t1 - t0) > REFRESH_INTERVAL) {

            // retrieve information to display
            freq = dhdGetComFreq();
            t0 = t1;

            // write down position
            if (dhdGetPosition(&px, &py, &pz) < 0) {
                done = 1;
            }

            px = px * 1000;
            py = py * 1000;
            pz = pz * 1000;

            //get the square of each coordinate of the haptic device
            int codx = pow(px, 2);
            int cody = pow(py, 2);
            int codz = pow(pz, 2);

            //calculate the radius if the spherical evelope
            double r = sqrt(cody + codz + codx);

            //set the forces to low when the spherical radius is less than 5 otherwise enable the joystick to be moved only along one axis,
            if (((px < 5) && (px > -5)) && ((py < 5) && (py > -5)) && ((pz < 5) && (pz > -5))) {
                Kz = 220;
                Ky = 220;
                Kx = 220;
            }
            //move joysick only along Z axis
            else if (((px < 5) && (px > -5)) && ((py < 5) && (py > -5))) {
                Kz = 220 - 2.5 * r;
                Ky = 1000;
                Kx = 1000;
            }
            //move joysick only along X axis
            else if (((py < 5) && (py > -5)) && ((pz < 5) && (pz > -5))) {
                Ky = 1000;
                Kx = 220 - 2.5 * r;
                Kz = 1000;
            }
            //move joysick only along Y axis
            else if (((px < 5) && (px > -5)) && ((pz < 5) && (pz > -5))) {
                Kx = 1000;
                Kz = 1000;
                Ky = 220 - 2.5 * r;
            }

            //set the forces to high when the spherical radius is higher than 5 and when neither of the cordinates are between -5 to +5,
            
            //update the files with the read data
            std::ofstream pipe("mypipe");

            //assign the position to the pipe and set 0 at all cordinates regardless the calibration error
            if ((py < 5) && (py > -5)) {
                pipey = 0;
            }
            else {
                pipey = py;
            }

            if ((pz < 5) && (pz > -5)) {
                pipez = 0;
            }
            else {
                pipez = pz;
            }

            if ((px < 5) && (px > -5)) {
                pipex = 0;
            }
            else {
                pipex = px;
            }

            //get the button reading and assign it to the pipe
            std::string result = std::to_string(pipex) + " " + std::to_string(pipey) + " " + std::to_string(pipez) + " " + std::to_string(dhdGetButtonMask());
            pipe << result;
            pipe.close();

            // user input
            if (dhdKbHit() && dhdKbGet() == 'q') done = 1;

        }
    }

    // close the connection and exit
    dhdClose();
    return 0;
}

