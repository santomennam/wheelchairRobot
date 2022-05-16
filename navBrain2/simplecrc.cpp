#include "simplecrc.h"

using namespace std;

unsigned char computeCRC8(const unsigned char  *bytes, int len) {
  const unsigned char  generator = 0b00101111;   // polynomial = x^8 + x^5 + x^3 + x^2 + x + 1 (ignore MSB which is always 1)
  unsigned char  crc = 0;

  while (len--)
  {
    crc ^= *bytes++; /* XOR-in the next input byte */

    for (int i = 0; i < 8; i++)
    {
      if ((crc & 0x80) != 0)
      {
        crc = (unsigned char )((crc << 1) ^ generator);
      }
      else
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}

void computeCRC8(const unsigned char  *bytes, int len, char& c1, char& c2)
{
    int crc = computeCRC8(bytes, len);
    c2 = digitToHexChar(crc & 0x0F);
    c1 = digitToHexChar(crc >> 4);
}

bool validateCRC8(const unsigned char *bytes, int len, char c1, char c2)
{
    int crc = computeCRC8(bytes, len);
    return (c2 == digitToHexChar(crc & 0x0F)) && (c1 == digitToHexChar(crc >> 4));
}

std::string addCRC8(std::string str)
{
    char crc[] = "xx:";
    computeCRC8(reinterpret_cast<const unsigned char*>(str.c_str()), str.size(), crc[0], crc[1]);
    string result(crc);
    result.append(str);
    return result;
}

bool stripCRC8(std::string& str)
{
    if (str.size() < 3) {
        return false;
    }
    bool valid = validateCRC8(reinterpret_cast<const unsigned char*>(str.data())+3, str.size()-3, str[0], str[1]);
    if (valid) {
        str = str.substr(3);
    }
    return valid;
}

//void testCRC8(std::string str)
//{
//    string s = addCRC8(str);
//    cout << s << endl;
//    if (stripCRC8(s)) {
//        if (s != str) {
//            cout << "Failed cr8 check roundtrip for '" << str << "' != '" << s << "'" << endl;
//            return;
//        }
//    }
//    else {
//        cout << "stripCRC8 returned false for '" << s << "'" << endl;
//        return;
//    }
//    cout << "Success: '" << str << "'" << endl;
//}

// adds: #XX:   before and \n after  where XX is CRC8
string wrapDelimitedCRC8(std::string str)
{
    char crc[] = "#xx:";
    computeCRC8(reinterpret_cast<const uint8_t*>(str.c_str()), str.length(), crc[1], crc[2]);
    string result(crc);
    result.append(str);
    result.append(1, '\n');
    return result;
}
