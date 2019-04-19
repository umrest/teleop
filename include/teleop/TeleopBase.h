/** 
 * @file TeleopBase.h
 * @brief Class definition for TeleopBase
 * @author Jack Shelata
 * @date April 18, 2019
 */

#include <bitset>
#include <cassert>

/***************************************************************
**                Class Definition
****************************************************************/

/** 
 * Teleop Base Class. This class serves as the base class for the teleop
 * classes. This class provides the basic storage and retrieval methods
 * for the 32-bit unsigned integers to send over ROS.
 */ 
class TeleopBase{
  public:
    /**
     * TeleopBase constructor.
     * Initializes data variable to 0.
     */
    TeleopBase();
    /**
     * Member function returns data value.
     * @return unsigned int data
     */
    unsigned int getData();
    /**
     * Member function sets data value.
     * @param data_in - unsigned int value to set data equal to
     */
    void setData(unsigned int data_in);
    /**
     * Member function sets desired bits in data.
     * @param bitLength - Length in bits of the value to set
     * @param offset - Offset in bits of value to set (between 0 and 32)
     * @param value - Value to set
     */
    void setBits(int bitLength, int offset, int value);
    /**
     * Member function to return desired bits as an Integer.
     * @param bitLength - Length in bits of the value to get
     * @param offset - Offset in bits of value to set (between 0 and 32)
     * @param isSigned - Set to true if value to get is signed
     */
    int getBits(int bitLength, int offset, bool isSigned);
  private:
    /**
     * Private variable to store data.
     * Stores data as a 32-bit unsigned integer.
     */
    unsigned int data;
};

/**
 * Helper function to create a mask.
 * Creates a mask of desired bitLength and offset.
 * @param bitLength - Length in bits of mask
 * @param offset - length in bits of offset
 */
unsigned int createMask(int bitLength, int offset);
/**
 * Helper function to sign a number.
 * @param bitLength - Length in bits of number to sign
 * @param value - Reference to value which will be signed
 */
void sign(int bitLength, int& value);
