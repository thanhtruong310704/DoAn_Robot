#ifndef BIEN_H
#define BIEN_H

#define ReLay 3
typedef struct
{
  float x;
  float y;
  float z;
} ToaDo;

typedef enum 
{
  WAIT,
  MOVING_TO_OBJ,
  PICK_OBJ,
  MOVING_TO_BOX,
  PLACING_OBJ
} StateRobot;

#endif