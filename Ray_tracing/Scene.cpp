#include "stdafx.h"
#include "Scene.h"
void line_(std::string line, std::string &name, std::vector<double> &attr)
{
	std::cout << line << std::endl;
	int type_begin = line.find("<");
	int type_end = line.find(" ", type_begin + 1);
	std::cout << line.substr(type_begin + 1, type_end - type_begin - 1) << " ";
	name = line.substr(type_begin + 1, type_end - type_begin - 1);
	int value_begin = line.find("\"", type_begin + 1);
	int value_end = line.find("\"", value_begin + 1);
	std::cout << value_begin << " " << value_end << std::endl;
	while (value_begin < value_end)
	{
		int one_value_end = line.find(",", value_begin + 1);
		if (one_value_end == -1) one_value_end = value_end;
		double one_value = stod(line.substr(value_begin + 1, one_value_end - value_begin - 1));
		attr.push_back(one_value);
		std::cout << one_value << " ";
		value_begin = one_value_end;
	}
	std::cout << std::endl;
}