#ifndef MACROS_H
#define MACROS_H

//#define ENABLE_PRINT

#ifdef ENABLE_PRINT
  #define Sprintln(args...) (Serial.println(args))
  #define Sprint(args...) (Serial.print(args))
#else
  #define Sprintln(args...) 
  #define Sprint(args...)
#endif

#endif
