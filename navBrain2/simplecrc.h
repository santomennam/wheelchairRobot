#ifndef SIMPLECRC_H
#define SIMPLECRC_H

#include <string>

// single digit 0-15 -> 0-F  (capital)
inline constexpr char digitToHexChar(int i)
{
    return i > 9 ? ('A' + i - 10) : '0' + i;
}

// single char 0-F (capital!!) to integer 0-15
inline constexpr int hexCharToDigit(char c)
{
    return c >= 'A' ? (c - 'A' + 10) : c - '0';
}

unsigned char computeCRC8(const unsigned char  *bytes, int len);
void computeCRC8(const unsigned char  *bytes, int len, char& c1, char& c2);
bool validateCRC8(const unsigned char *bytes, int len, char c1, char c2);
std::string addCRC8(std::string str);
std::string wrapDelimitedCRC8(std::string str);
bool stripCRC8(std::string& str);
void testCRC8(std::string str);

#endif // SIMPLECRC_H
