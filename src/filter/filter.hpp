//------------------------------------------------------------------------------
//   TumanakoVC - Electric Vehicle and Motor control software
//   Copyright (C) 2010 Philip Court <philip@greenstage.co.nz>
//
//   This file is part of TumanakoVC.
//
//   TumanakoVC is free software: you can redistribute it and/or modify
//   it under the terms of the GNU Lesser General Public License as published
//   by the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
//   TumanakoVC is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//   GNU Lesser General Public License for more details.
//
//   You should have received a copy of the GNU Lesser General Public License
//   along with TumanakoVC.  If not, see <http://www.gnu.org/licenses/>.
//
// DESCRIPTION:
//   Filters digital data by storing historical data in a buffer. Implements
//   a simple only ON if all ON type logic. 
//
// HISTORY:
//   Philip Court 15/Nov/2010 - First Cut
//------------------------------------------------------------------------------
#include <iostream>

using namespace std;


// Used to filter noisy digital lines.  Will only
// return true if line has been continuously on for
// the given number of values in the constructor
class filter {
public:

  //Constructor, specifies the size of the filter
  filter (unsigned long size) {
    index_=0;
    size_ = size;
    buffer_ = new bool[size];

    //initialise
    for (unsigned short i=0; i<size_; i++)
      buffer_[i] = false;
  }

  //Store data in the filters buffer
  void store(bool data) {
    buffer_[index_] = data;
    if (++index_ == size_) index_ = 0; //increment and loop back around
  }

  //Get the result (true if all data in the buffer is true, otherwise false)
  bool result() {
    for (unsigned short i=0; i<size_; i++) {
      if (buffer_[i] == false) return false;
    }
    return true;
  }

  //Returns the percentage of values in the buffer that don't match the actual reported result (i.e. noise)
  unsigned short percentageNoise() {
    unsigned short count = 0;
    bool actualResult = result();
    for (unsigned short i=0; i<size_; i++) {
      if (buffer_[i] != actualResult) count++;
    }
    return (100*count/size_);
  }

private:
  bool * buffer_;
  unsigned short index_;
  unsigned short size_;
};

