#ifndef __CORE_TIMER_H__
#define __CORE_TIMER_H__

#include <string>
#include <iostream>

class Timer
{
private:
	int	running;
	double	last;
	double	runn,elap;

	double now();
	void update();

	std::string format() const;

public:
	Timer();

	void start();
	void stop();
	void reset();
	void restart();

	double runned();
	double elapsed();

	friend std::ostream& operator<<(std::ostream& os,const Timer& ts);

};

std::ostream& operator<<(std::ostream& os,const Timer& ts);

#endif // __CORE_TIMER_H__ 

