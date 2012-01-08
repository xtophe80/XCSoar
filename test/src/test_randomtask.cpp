/* Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2012 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#include "Math/FastMath.h"
#include "harness_flight.hpp"

int main(int argc, char** argv) 
{
  // default arguments
  autopilot_parms.ideal();

  if (!parse_args(argc,argv)) {
    return 0;
  }

#define NUM_TESTS 2

  plan_tests(NUM_TESTS);

  for (int j=0; j<NUM_TESTS; j++) {
    unsigned i = rand()%NUM_WIND;

    if (j+1==NUM_TESTS) {
      verbose=1;
    }
    ok (test_flight_times(7,i), test_name("flight times",7,i),0);
  }
  return exit_status();
}
