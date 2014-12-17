/*
Copyright (c) 2013-2015, Gregory P. Meyer
                         University of Illinois Board of Trustees
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder(s) nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// Standard Libraries
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv) {
  if (argc != 3) {
    printf("Usage: %s <Standard Results> <Masking Results>\n", argv[0]);
    return -1;
  }

  FILE *standard = fopen(argv[1], "r");
  FILE *masking = fopen(argv[2], "r");


  if ((standard != NULL) && (masking != NULL)) {
    int sdetections = 0, struepos = 0;
    int mdetections = 0, mtruepos = 0, mfalsepos = 0, mfalseneg = 0;
    float stime = 0.0f, mtime = 0.0f;
    int frames = 0;

    while(true) {
      int sframe, spostive, snegative;
      int mframe, mpostive, mnegative;
      float stiming, mtiming;

      if (fscanf(standard, "%d: %d, %d, %f\n", &sframe, &spostive, &snegative,
                 &stiming) != 4) {
        break;
      }

      if (fscanf(masking, "%d: %d, %d, %f\n", &mframe, &mpostive, &mnegative,
                 &mtiming) != 4) {
        break;
      }

      if (sframe != mframe) {
        printf("[Error] Invalid frame number.\n");
        break;
      }

      sdetections += (spostive + snegative);
      struepos += spostive;

      mdetections += (mpostive + mnegative);
      mtruepos += mpostive;
      mfalsepos += mnegative;
      mfalseneg += (spostive - mpostive);

      stime += stiming;
      mtime += mtiming;

      frames++;
    }

    printf("%f,%f,%f,%f,%f,%f\n",
           (float)struepos / sdetections,
           (float)struepos / (struepos + 0),
           stime / frames,
           (float)mtruepos / mdetections,
           (float)mtruepos / (mtruepos + mfalseneg),
           mtime / frames);

    fclose(standard);
    fclose(masking);
  } else {
    printf("[Error] Could not open files.\n");
  }

  return 0;
}
