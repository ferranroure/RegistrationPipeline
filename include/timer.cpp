#include "timer.h"

#include <time.h>
#include <sys/time.h>
#include <sys/times.h>
#include <sstream>
#include <iomanip>
#include <stdint.h>

using namespace std;

Timer::Timer()
{
	reset();
}

double Timer::now()
{
	double now_s;

	struct timeval tv;
	gettimeofday(&tv,0);
	now_s = tv.tv_sec + 1e-6*tv.tv_usec;

	return now_s;
}

void Timer::start()
{
	running = 1;
	update();
}

void Timer::stop()
{
	update();
	running = 0;
}

void Timer::reset()
{
	last = now();
	runn = elap = 0;
	running = 0;
	update();
}

void Timer::restart()
{
	reset();
	start();
}

double Timer::runned()
{
	update();
	return runn;
}

double Timer::elapsed()
{
	update();
	return elap;
}

void Timer::update()
{
	double now_new = now();
	double delta = now_new - last;
	last = now_new;
	elap += delta;
	if (running)
		runn += delta;
}

std::string Timer::format() const
{
	int32_t y,d,h,mn,s,ms,us;
	uint64_t tmp = uint64_t(runn*1e6);

	y =  tmp/(365*24*60*60*1000000ULL);
	tmp %=   (365*24*60*60*1000000ULL);
	d =  tmp/(24*60*60*1000000ULL);
	tmp %=   (24*60*60*1000000ULL);
	h =  tmp/(60*60*1000000ULL);
	tmp %=   (60*60*1000000ULL);
	mn = tmp/(60*1000000ULL);
	tmp %=   (60*1000000ULL);
	s =  tmp/1000000ULL;
	tmp %=   1000000ULL;
	ms = tmp/1000ULL;
	us = tmp%1000ULL;

	std::ostringstream ss;

	if (y)
	ss << setfill('0') << setprecision(3) << setw(3) << y  << ':';
	if (y && d)
	ss << setfill('0') << setprecision(3) << setw(3) << d  << ':';
	ss << setfill('0') << setprecision(2) << setw(2) << h  << ':';
	ss << setfill('0') << setprecision(2) << setw(2) << mn  << ':';
	ss << setfill('0') << setprecision(2) << setw(2) << s;
	ss << '.' << setfill('0') << setprecision(3) << setw(3) << ms;
	ss << setfill('0') << setprecision(3) << setw(3) << us;
	ss << ends;

	return ss.str();
}

std::ostream& operator<<(std::ostream& os,const Timer& ts)
{
	os << "[" << ts.format() << "]";
	return os;
}

