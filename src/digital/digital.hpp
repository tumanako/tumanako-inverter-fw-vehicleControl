//------------------------------------------------------------------------------
//   TumanakoVC - Electric Vehicle and Motor control software
//   Copyright (C) 2012 Philip Court <philip@greenstage.co.nz>
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
//------------------------------------------------------------------------------


#ifndef __DIGITAL_H
#define __DIGITAL_H

//Stores digital and enginerring unit (EU) ranges for easy conversion of digital data to EU.
//Digital units are unsigned short (0 -> 65535)
//EU inits are short (-32768 -> 32767)
//TODO template for these data types would be useful
class Digital {
public:
  Digital(unsigned short digital_min, unsigned short digital_max, short eu_min, short eu_max) { 
    digital_range_ = digital_max - digital_min;
    eu_range_ = eu_max - eu_min;
    eu_min_ = eu_min;
    digital_min_ = digital_min;
      
    //Caluclate constants for later use
    gradient_ = (float)eu_range_ / digital_range_;
    offset_ = (float)eu_min_ - (gradient_ * digital_min_);
  }

  short eu(unsigned short digital) {
    return (round(digital * gradient_ + offset_));
  }
  
  //Convert to Engineering Units (EU), but limit the EU upper range by 'eu_limit' during the conversion
  short eu(unsigned short digital, short eu_limit) {
    //if (eu_limit>eu_range_) return 0;  //TODO temp hack
    
    //adjust local varibles so we don't cause side effects
    float gradient = (float)(eu_range_ - eu_limit) / digital_range_;
    float offset = (float)eu_min_ - (gradient * digital_min_);

    return (round(digital * gradient + offset));
  }

private:
  short round(float eu) {
    if (eu >= 0) return (short)(eu + 0.5);  //+ve, hence round up (i.e. away from zero)
    else return (short)(eu - 0.5); //-ve, hence round down (i.e. still away from zero)
  }
  
  float gradient_;  //TODO convert to Q Number
  float offset_;
  short eu_min_;
  short eu_range_;
  unsigned short digital_range_;
  unsigned short digital_min_;
};  

#endif //__DIGITAL_H
