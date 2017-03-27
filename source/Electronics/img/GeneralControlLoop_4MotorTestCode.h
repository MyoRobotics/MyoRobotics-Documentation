/*
 * GeneralControlLoop.h
 *
 *  Created on: 13 Oct 2014
 *      Author: alex
 */

#ifndef GENERALCONTROLLOOP_H_
#define GENERALCONTROLLOOP_H_

#include <ral/IMuscle.h>
#include <ral/IRobot.h>
#include "robot/CommunicationData.h"
#include <qmutex.h>

#include <QObject>

#include <QTimer>
#include <QMutex>
#include <QThread>
#include <QtCore/QObject>

#include <QObject>
#include <QLabel>
#include <QTimer>
#include <QMutex>
#include <QThread>
#include <QMap>
#include <QFile>
#include <QObject>

#include <QTimer>
#include <QMutex>
#include <QThread>
#include <QMap>
#include <QFile>

#include <stdint.h>

#include <QElapsedTimer>

#include <IGeneralControlLoop.h>
#include <ControlParameter.h>

namespace eu
{
namespace myode
{
namespace myorobot
{

class GeneralControlLoop : public IGeneralControlLoop
{
	Q_OBJECT

public:
    GeneralControlLoop(IRobot* robot);
    virtual ~GeneralControlLoop();

    virtual void cycle();
    virtual void init();


private:
   ControlParameter* motor0Test;
   ControlParameter* motor1Test;
   ControlParameter* motor2Test;
   ControlParameter* motor3Test;
};
}
}
}
#endif /* GENERALCONTROLLOOP_H_ */
