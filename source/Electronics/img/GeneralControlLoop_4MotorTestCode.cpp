/*
 * GeneralControlLoop.cpp
 *
 *  Created on: 13 Oct 2014
 *      Author: alex
 */

#include "GeneralControlLoop.h"
#include <stdio.h>
#include <QObject>
#include <stdio.h>
#include <QTimer>
#include <QMutex>
#include <QThread>
#include <iostream>
#include "robot/CommunicationData.h"
#include <math.h>
#include "ControlParameter.h"

namespace eu
{
namespace myode
{
namespace myorobot
{

GeneralControlLoop::GeneralControlLoop(IRobot* robot):IGeneralControlLoop(robot,100){}

GeneralControlLoop::~GeneralControlLoop(){
    delete motor0Test;
    delete motor1Test;
    delete motor2Test;
    delete motor3Test;

}

void GeneralControlLoop::init()
{
		motor0Test = new ControlParameter("M 0",p_robot);
		motor1Test = new ControlParameter("M 1",p_robot);
		motor2Test = new ControlParameter("M 2",p_robot);
		motor3Test = new ControlParameter("M 3",p_robot);

        std::cout<<"Setting Control Parameters"<<std::endl;
		localParameters.outputNegMax=-4000;  //4000 is 100%
		localParameters.outputPosMax=4000;
		localParameters.polyPar[0]=0; //adjust those 4 values to your spring //should map displacement into a force in [N]
		localParameters.polyPar[1]=1;
		localParameters.polyPar[2]=0;
		localParameters.polyPar[3]=0;

/* experimentally determined (0N - 300N, least squares)
 *                              polyPar[0]  polyPar[1]  polyPar[2]  polyPar[3]
 *      spring 1 [weakest]      0.621249    0.237536    -0.000032   0.000000
 *      spring 2                1.604382    0.508932    -0.000117   0.000000
 *      spring 3 [strongest]    1.186483    0.938377    -0.000274   0.000001
 */

		localParameters.radPerEncoderCount=2*M_PI/(2000.0*53.0); //gear box ratio of 53 and 2000 counts per rotation with encoder (4 x 1000, edge detection)
		localParameters.spNegMax=-1000;
		localParameters.spPosMax=1000;
		localParameters.tag=0; //don't care, leave to 0
		localParameters.timePeriod=1010; //us
		localParameters.torqueConstant=1; //TODO correct this

		localParameters.params.pidParameters.IntegralNegMax=-4000;
		localParameters.params.pidParameters.IntegralPosMax=4000;
		localParameters.params.pidParameters.deadBand=0;
		localParameters.params.pidParameters.forwardGain=0;
        localParameters.params.pidParameters.pgain=0.12;
		localParameters.params.pidParameters.igain=0;
		localParameters.params.pidParameters.dgain=0;
		localParameters.params.pidParameters.integral=0; //initialise the integrator
		localParameters.params.pidParameters.lastError=0; //initialise last error

        p_robot->getGanglion(0)->getMuscles()[0]->setControllerParams(Raw,localParameters); //force control: use Force, make sure to adjust pgain above, suggested 10 for now
        p_robot->getGanglion(0)->getMuscles()[1]->setControllerParams(Raw,localParameters);

        p_robot->getGanglion(0)->getMuscles()[2]->setControllerParams(Raw,localParameters); //force control: use Force, make sure to adjust pgain above, suggested 10 for now
        p_robot->getGanglion(0)->getMuscles()[3]->setControllerParams(Raw,localParameters);
        /*p_robot->getGanglion(0)->getMuscles()[2]->setControllerParams(Force,localParameters);
        p_robot->getGanglion(0)->getMuscles()[3]->setControllerParams(Force,localParameters);*/

	}

void GeneralControlLoop::cycle()
{
	if (! p_robot->controlparameterRequestQueueEmpty())
	{
		//we should not run the controllers if there are still parameter requests in the queue
      //  std::cout<<"Serving Queue."<< targetForce->get() <<std::endl;
	}
	else
	{
		double referencePosition=0;
		double maxDelta=100;
		double baseForce=100;

			p_robot->getGanglion(0)->getMuscles()[0]->enableController();
            p_robot->getGanglion(0)->getMuscles()[1]->enableController();
			p_robot->getGanglion(0)->getMuscles()[2]->enableController();
            p_robot->getGanglion(0)->getMuscles()[3]->enableController();
            /*p_robot->getGanglion(0)->getMuscles()[2]->enableController();
            p_robot->getGanglion(0)->getMuscles()[3]->enableController();*/

            double jointPositionUpper=p_robot->getGanglion(0)->getJoints()[0]->getAngle();

			double deltaForce=(referencePosition - jointPositionUpper) *200;

			if (deltaForce>maxDelta)
			{
				deltaForce=maxDelta;

			}
			if (deltaForce<-maxDelta)
			{
				deltaForce=-maxDelta;
			}

			//p_robot->getGanglion(0)->getMuscles()[0]->setControllerRef(Force,baseForce+deltaForce); //pulls in positive joint direction
			//p_robot->getGanglion(0)->getMuscles()[3]->setControllerRef(Force,baseForce-deltaForce); //pulls in negative joint direction

            p_robot->getGanglion(0)->getMuscles()[0]->setControllerRef(Raw,motor0Test->get());
            p_robot->getGanglion(0)->getMuscles()[1]->setControllerRef(Raw,motor1Test->get());
            p_robot->getGanglion(0)->getMuscles()[2]->setControllerRef(Raw,motor2Test->get());
            p_robot->getGanglion(0)->getMuscles()[3]->setControllerRef(Raw,motor3Test->get());
            /*p_robot->getGanglion(0)->getMuscles()[2]->setControllerRef(Force,targetForce->get());
            p_robot->getGanglion(0)->getMuscles()[3]->setControllerRef(Force,targetForce->get());//*/
            //TODO: check access to jointAngle!
			//jointPositionUpper=p_robot->getGanglion(0)->getJoints()[0]->	//jointPositionLower=p_robot->getGanglion(0)->getJoints()[1];

    }
}



}

}
}


