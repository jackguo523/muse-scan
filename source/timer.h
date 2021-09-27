// high resolution cpu clock based on the chrono library

#ifndef TIMER_H
#define TIMER_H

#include <chrono>
#include <iostream>

static std::chrono::high_resolution_clock::time_point start;
static std::chrono::high_resolution_clock::time_point stop;

// timer start
static void timer_start() {
	start = std::chrono::high_resolution_clock::now();
}

// timer end in different units
template<typename T>
static T timer_stop() {
	stop = std::chrono::high_resolution_clock::now();
	T duration = std::chrono::duration_cast<T>(stop - start);
	return duration;
}

#endif