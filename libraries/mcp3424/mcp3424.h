#ifndef mcp3424_h
#define mcp3424_h

#include "Arduino.h"


class mcp3424
{
	public:
	mcp3424();
	void GetAddress(char Addr0, char Addr1);
	float GetValue(int channel);

	private:
	char address;

};

#endif