// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <stdio.h>
#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <string>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;

int main(int argc, char *argv[]) 
{
    Network yarp;

    Property params;
    params.fromCommand(argc, argv);

    if (!params.check("robot"))
    {
        fprintf(stderr, "Please specify the name of the robot\n");
        fprintf(stderr, "--robot name (e.g. icub)\n");
        return 1;
    }
    
    //robot name
    std::string robotName=params.find("robot").asString().c_str();
    
    //right arm
    std::string remotePortsR="/";
    remotePortsR+=robotName;
    remotePortsR+="/right_arm";
    
    
    //left arm
    std::string remotePortsL="/";
    remotePortsL+=robotName;
    remotePortsL+="/left_arm";
    
    
    std::string localPorts="/test/client";

    Property optionsR, optionsL;
    
    optionsL.put("device", "remote_controlboard");
    optionsL.put("local", localPorts.c_str());   //local port names
    optionsL.put("remote", remotePortsL.c_str());         //where we connect to
    
    optionsR.put("device", "remote_controlboard");
    optionsR.put("local", localPorts.c_str());   //local port names
    optionsR.put("remote", remotePortsR.c_str());         //where we connect to
    
    

    // create a device
    PolyDriver robotDeviceR(optionsR);
    if (!robotDeviceR.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return 0;
    }
    // create a device
    PolyDriver robotDeviceL(optionsL);
    if (!robotDeviceL.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return 0;
    }

    IPositionControl *posR;
    IEncoders *encsR;
    
    IPositionControl *posL;
    IEncoders *encsL;

    bool ok;
    
    ok = robotDeviceR.view(posR);
    ok = ok && robotDeviceR.view(encsR);

    if (!ok) {
        printf("Problems acquiring right interfaces\n");
        return 0;
    }
    
    ok = robotDeviceL.view(posL);
    ok = ok && robotDeviceL.view(encsL);
    
    if (!ok) {
        printf("Problems acquiring left interfaces\n");
        return 0;
    }

    int nj=0;
    posR->getAxes(&nj);
    Vector encodersR;
    Vector encodersL;
    Vector command;
    Vector tmp;
    encodersR.resize(nj);
    encodersL.resize(nj);
    tmp.resize(nj);
    command.resize(nj);
    
    int i;
    for (i = 0; i < nj; i++) {
         tmp[i] = 50.0;
    }
    //pos->setRefAccelerations(tmp.data());

    for (i = 0; i < nj; i++) {
        tmp[i] = 1.0;
        posR->setRefSpeed(i, tmp[i]);
        posL->setRefSpeed(i, tmp[i]);
        
    }

    posR->setRefSpeeds(tmp.data());
    posL->setRefSpeeds(tmp.data());
    
    //fisrst read all encoders
    //
    printf("waiting for Right encoders");
    while(!encsR->getEncoders(encodersR.data()))
    {
        Time::delay(0.1);
        printf(".");
    }
    printf("\n;");
    printf("waiting for Left encoders");
    while(!encsL->getEncoders(encodersL.data()))
    {
        Time::delay(0.1);
        printf(".");
    }
    printf("\n;");

    command=encodersR;
    //now set the shoulder to some value
    command[0]=-50;
    command[1]=20;
    command[2]=-10;
    command[3]=50;
    posR->positionMove(command.data());
    //posL->positionMove(command.data());
    
    bool doneR=false;
    bool doneL=false;

    while(!(doneR && doneL))
    {
        posR->checkMotionDone(&doneR);
        posL->checkMotionDone(&doneL);
        Time::delay(0.1);
    }

    int times=0;
    while(true)
    {
        times++;
        if (times%2)
        {
             command[0]=-50;
             command[1]=20;
             command[2]=-10;
             command[3]=50;
        }
        else
        {
             command[0]=-20;
             command[1]=40;
             command[2]=-10;
             command[3]=30;
        }

        posR->positionMove(command.data());
        //posL->positionMove(command.data());
        
        int count=50;
        while(count--)
            {
                Time::delay(0.1);
                bool ret=encsR->getEncoders(encodersR.data());
                
                if (!ret)
                {
                    fprintf(stderr, "Error receiving encoders, check connectivity with the robot\n");
                }
                else
                { 
                    printf("%.1lf %.1lf %.1lf %.1lf\n", encodersR[0], encodersR[1], encodersR[2], encodersR[3]);
                }
            }
    }

    robotDeviceR.close();
    robotDeviceL.close();
    
    return 0;
}
