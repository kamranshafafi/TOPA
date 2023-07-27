#ifndef COORDINATES_H
#define COORDINATES_H

#include <fstream>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

class Coords
{
  public:
    Coords()
    {
    }

    Coords(float x, float y, float z)
        : x{x},
          y{y},
          z{z}
    {
    }

    float x{}, y{}, z{};
};

using CoordsList = std::vector<Coords>;

CoordsList loadCoordsFromFile(const std::string& path);
void dumpCoords(std::ostream& os, const CoordsList& coordsList, const std::string& title);

#endif // COORDINATES_H