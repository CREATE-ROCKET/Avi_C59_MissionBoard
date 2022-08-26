/*
 Author : Yuta Takayasu
 Date : 2022/8/5

    This program is Process Interface.
    
*/


#pragma once

#ifndef PI_H
#define PI_H

class ProcessInterface
{
   public:
   virtual void begin() = 0;
   virtual void tickProcess() = 0;
    
};
#endif