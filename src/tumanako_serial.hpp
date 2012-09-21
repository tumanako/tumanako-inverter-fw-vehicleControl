//------------------------------------------------------------------------------
//   TumanakoVC - Electric Vehicle and Motor control software
//   Copyright (C) 2009 Philip Court <philip@greenstage.co.nz>
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
//------------------------------------------------------------------------------

// Prepare RS323 port for writing
void usartInit();

//Write 5 longs to port
void usartWrite(long a, long b, long c, long d, long e);

//write null terminated sequence of chars
void usartWriteChars(const char * chars);

//------------------------------------------------------------------------------
//write variable number of parameters
//Format Options:
// f double (1dp)
// F double (4dp)
// i int (use min 3 chars)
// I int (use min 5 chars)
// b boolean
// s string (char *)
//------------------------------------------------------------------------------
void printFormat( const char* Format, ... );

void usartWriteDisclaimer();
