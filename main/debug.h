#ifndef DEBUG_H
#define DEBUG_H

////////////Structures

struct DebugIKData{
  float x;
  float y;
  float z;

  float rTool;
  float rWrist;
  float zWrist;

  float rp;
  float zp;
  float D;

  float degA;
  float degB;
  float degZ;

  float cmdA;
  float cmdB;
  float cmdZ;

  bool reachable;
};


/////////////Prototypes

void printPoint(const char* label, const Coordinates& p);
void printIKDebug(const DebugIKData& d);


#endif