//
// Created by jun on 8/15/17.
//
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include "map.h"


Map::Map() {}

Map::~Map() {}

//
// Reads map data from a file.
//
// @param filename: Name of file containing map data.
// @param map: Map object
// @return: True if opening and reading file was successful
//
void Map::readData(std::string filename) {
  std::ifstream in_file_map(filename.c_str(), std::ifstream::in);

  if (!in_file_map) {
    throw std::runtime_error("Could not open map file!");
  }

  std::string line_map;

  while(getline(in_file_map, line_map)) {
    std::istringstream iss_map(line_map);

    Landmark landmark;

    iss_map >> landmark.x;
    iss_map >> landmark.y;
    iss_map >> landmark.id;

    landmark_list_.push_back(landmark);
  }
}

std::vector<Landmark> Map::getLandmarkList() const { return landmark_list_; }
