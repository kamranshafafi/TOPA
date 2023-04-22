#include "coords.h"

#include <iostream>

CoordsList
loadCoordsFromFile(const std::string& path)
{
    CoordsList coordsList;
    std::ifstream ifs{path};
    std::string line;

    if (ifs.is_open())
    {
        while (std::getline(ifs, line))
        {
            std::string x, y, z;
            std::istringstream iss{line};
            std::getline(iss, x, ',');
            std::getline(iss, y, ',');
            std::getline(iss, z);
            coordsList.emplace_back(std::stof(x), std::stof(y), std::stof(z));
        }
    }
    else
    {
        std::cout << "File not found" << std::endl;
    }
    ifs.close();

    return coordsList;
}

void
dumpCoords(std::ostream& os, const CoordsList& coordsList, const std::string& title)
{
    if (!title.empty())
    {
        os << title << "\n";
    }

    for (const Coords& c : coordsList)
    {
        os << "{" << c.x << ", " << c.y << ", " << c.z << "}\n";
    }

    os << std::endl;
}