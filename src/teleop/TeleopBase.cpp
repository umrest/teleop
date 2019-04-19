/** 
 * @file TeleopBase.cpp
 * @brief Class implementation for TeleopBase.
 * @author Jack Shelata
 * @date April 18, 2019
 */

#include "TeleopBase.h"

/***************************************************************
**                Functions Definition
****************************************************************/

TeleopBase::TeleopBase(){
  data = 0;
}

unsigned int TeleopBase::getData(){
  return data;
}

void TeleopBase::setData(unsigned int data_in){
  data = data_in;
}

void TeleopBase::setBits(int bitLength, int offset, int value){
  assert(bitLength > 0 && bitLength < 32);
  assert(offset >= 0 && offset < 32);

  // create a mask
  unsigned int mask = createMask(bitLength, offset);

  // shift value
  value <<= offset;

  // remove excess bits from value
  value &= mask;

  // invert mask for set
  mask = ~mask;

  // remove existing bits
  data &= mask;

  // shift and set bits
  data |= value;
}

int TeleopBase::getBits(int bitLength, int offset, bool isSigned){
  assert(bitLength > 0 && bitLength < 32);
  assert(offset >= 0 && offset < 32);

  // create a mask
  unsigned int mask = createMask(bitLength, offset);

  // strip the desired bits
  int value = data & mask;

  // shift
  value >>= offset;

  // if the value should be signed, call sign
  if(isSigned) sign(bitLength, value);

  return value;
}

unsigned int createMask(int bitLength, int offset){
  assert(bitLength > 0 && bitLength < 32);
  assert(offset >= 0 && offset < 32);
  unsigned int mask = 0;
  for(int i = 0; i < bitLength; ++i){
    mask |= 1UL << i;
  }
  mask <<= offset;
  return mask;
}

void sign(int bitLength, int& value){
  assert(bitLength > 0 && bitLength < 32);
  int MSB = value >> (bitLength - 1);
  if(!MSB){
    return;
  }
  unsigned int mask = ~createMask(bitLength, 0);
  value |= mask;
}
