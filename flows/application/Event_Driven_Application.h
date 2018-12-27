/*
 * EventDrivenApplication.h
 *
 *  Created on: 17 nov. 2018
 *      Author: hajer
 */

#ifndef FLOWS_APPLICATION_EVENT_DRIVEN_APPLICATION_H_
#define FLOWS_APPLICATION_EVENT_DRIVEN_APPLICATION_H_



#include "Application.h"

class EventDrivenApplication : public Application {
public:
	EventDrivenApplication();
	virtual ~EventDrivenApplication();

	virtual void DoStart (void);
	virtual void DoStop (void);

    void
    ScheduleTransmit (double time);
    void
    Send (void);

    void
    SetSize(int size);
    int
    GetSize (void) const;
    void
    SetInterval(double interval);
    double
    GetInterval (void) const;

    //HB
    void SetLambda(int lambda);
    int GetLambda()const;
   void PoissonRand (void);
    //int fact(int nbr);

private:

	double m_interval;
	int m_size;
	int m_lambda;
};



#endif /* FLOWS_APPLICATION_EVENT_DRIVEN_APPLICATION_H_ */
