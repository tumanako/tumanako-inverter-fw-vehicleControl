//------------------------------------------------------------------------------
//   TumanakoVC - Electric Vehicle and Motor control software
//   Copyright (C) 2011 Philip Court <philip@greenstage.co.nz>
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

// This glue implements OS level hooks required for alocating memory and other good things

extern "C" {
#include <sys/stat.h>
#include <sys/types.h>

  extern int _stack; //check libopenstm32.ld

  //TODO: PCC - Not sure if these are safely implemented?
  void __exidx_start(int) {};
  void __exidx_end(int) {};
  //void exit(int a) {while (1){}};
  void abort() {
    while (1) {}};

  // Increase program data space (minimal implementation).
  // As malloc and related functions depend on this, it is useful (in our case essential) to have a working implementation.
  // The following suffices for a standalone system;
  // it exploits the symbol 'end' automatically defined by the GNU linker.
  caddr_t _sbrk(int incr) {
    extern char end;    // Defined by the linker
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == 0) {
      heap_end = &end;
    }
    prev_heap_end = heap_end;
    if (heap_end + incr > (char *)&_stack) {
      //_write (1, "Heap and stack collision\n", 25);  //TODO Gareth implemented this in libopenstm32
      abort ();
    }

    heap_end += incr;
    return (caddr_t) prev_heap_end;
  }

  //int _write(int a, char * b, int c){};

  //Close a file (minimal implementation).
  int _close(int) {
    return -1;
  };

  //Set position in a file (minimal implementation).
  //parameters: int file, int ptr, int dir
  int _lseek(int, int, int) {
    return 0;
  };

  //Read from a file (minimal implementation)
  //parameters: int file, char *ptr, int len
  int _read(int, char, int) {
    return 0;
  };

  //Status of an open file (minimal implementation).
  //parameters: int file, struct stat *st
  int _fstat(int, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
  }

  //Query whether output stream is a terminal (minimal implementation).
  //prameters: int file
  int _isatty(int) {
    return 1;
  }
}
