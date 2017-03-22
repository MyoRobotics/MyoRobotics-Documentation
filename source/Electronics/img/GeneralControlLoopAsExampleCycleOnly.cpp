
void GeneralControlLoop::cycle() //cyclic loop
{
	if (! p_robot->controlparameterRequestQueueEmpty())
	{
		//we should not run the controllers if there are still parameter set
		// requests in the queue
        std::cout<<"Serving Queue."<<endl;
	}
	else
	{
		    double referencePosition=0;

            //do some useful control here, e.g.
            
            double jointPositionUpper=p_robot->getGanglion(0)->getJoints()[2]->getAngle();
            double jointMotorVelocity=p_robot->getGanglion(3)->getMuscles()[3]->getMotorVelocity();
            
            
            
			p_robot->getGanglion(0)->getMuscles()[0]->enableController(); //enable 
            p_robot->getGanglion(0)->getMuscles()[0]->setControllerRef(Position, referencePosition); //set reference
            
            
			p_robot->getGanglion(3)->getMuscles()[2]->enableController(); //enable 
            p_robot->getGanglion(3)->getMuscles()[2]->setControllerRef(Position, referencePosition); //set reference


    }
}



