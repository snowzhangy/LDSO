#pragma once

#include <chrono>
#include <thread>
inline void usleep(int usec) { 
	std::this_thread::sleep_for(std::chrono::microseconds(usec));
}

inline void sleep(int sec) { std::this_thread::sleep_for(std::chrono::seconds(sec)); }
