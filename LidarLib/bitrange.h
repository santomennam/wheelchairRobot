#ifndef BITRANGE_H
#define BITRANGE_H



#include <limits>
#include <bitset>
#include <iostream>
#include <sstream>

// TODO  variadic templates

namespace twiddle {

    template <typename T>
    constexpr T abs(T x) {
        return x < T{} ? -x : x;
    }

    template <typename T>
    constexpr size_t bitCount() {
        return std::numeric_limits<typename std::make_unsigned<T>::type>::digits;
    }

    template <typename T>
    constexpr T allBitsSet()
    {
        return ~typename std::make_unsigned<T>::type{0};
    }

    template <int FROM_BIT, typename T>
    constexpr T maskUp()
    {
        return static_cast<T>(allBitsSet<T>() << FROM_BIT);
    }

    template <int FROM_BIT, typename T>
    constexpr T maskDown()
    {
        return static_cast<T>(allBitsSet<typename std::make_unsigned<T>::type>() >> (bitCount<T>()-FROM_BIT-1));
    }

    template <int HI_BIT, int LO_BIT, typename T>
    constexpr T mask()
    {
        return static_cast<T>(maskUp<LO_BIT,T>() & maskDown<HI_BIT,T>());
    }

    template <int HI_BIT, int LO_BIT, typename T>
    T mask(T value)
    {
        return static_cast<T>(value & mask<HI_BIT, LO_BIT, T>());
    }

    template <typename T>
    constexpr T invmask(int HI_BIT, int LO_BIT)
    {
        return ~mask<T>(HI_BIT, LO_BIT);
    }

    enum class BitRangeSrcType {
        isNormalizedAndMasked,  // value is assumed to be shifted so that LSB is bit zero, and that bits above the significant bits are 0
        isNormalizedNotMasked,  // value is assumed to be shifted so that LSB is bit zero, but bits outside of the specified range could be anything
        isMaskedNotNormalized,  // value is assumed to be in the appropriate position, and bits outside the range are assumed to be 0
        notNormalizedNotMasked  // value is assumed to be in the appropriate position, but bits outside the range could be anything
    };

    constexpr bool isNormalized(BitRangeSrcType srcType) {
        return (srcType == BitRangeSrcType::isNormalizedAndMasked) || (srcType == BitRangeSrcType::isNormalizedNotMasked);
    }

    constexpr bool isMasked(BitRangeSrcType srcType) {
        return (srcType == BitRangeSrcType::isNormalizedAndMasked) || (srcType == BitRangeSrcType::isMaskedNotNormalized);
    }

    constexpr bool isNotNormalized(BitRangeSrcType srcType) {
        return (srcType == BitRangeSrcType::isMaskedNotNormalized) || (srcType == BitRangeSrcType::notNormalizedNotMasked);
    }

    constexpr bool isNotMasked(BitRangeSrcType srcType) {
        return (srcType == BitRangeSrcType::isNormalizedNotMasked) || (srcType == BitRangeSrcType::notNormalizedNotMasked);
    }

    template <int HI_BIT, int LO_BIT, typename T>
    class BitRange {
    private:
        T value;
    public:
      constexpr BitRange(T val, BitRangeSrcType srcType = BitRangeSrcType::notNormalizedNotMasked)
            : value{static_cast<T>(isNotMasked(srcType) ? (isNotNormalized(srcType) ? (val & mask()) : ((val << LO_BIT) & mask())) : (isNotNormalized(srcType) ? (val) : (val << LO_BIT)))} {}
      template <int H1, int L1>
        constexpr BitRange(BitRange<H1,L1,T> v1, bool shouldMask) : value{shouldMask ? (v1.template shiftedToPosMasked<T, LO_BIT>() & mask()) : v1.template shiftedToPosNotMasked<T, LO_BIT>()}  {}
      static constexpr int numBits() { return HI_BIT-LO_BIT+1; }

      static constexpr T mask() { return twiddle::mask<HI_BIT, LO_BIT, T>(); }
      static constexpr T maskDown() { return twiddle::maskDown<HI_BIT, T>(); }
      static constexpr T maskUp() { return twiddle::maskUp<LO_BIT, T>(); }
      static constexpr T bitsAbove() { return ((HI_BIT+1) < bitCount<T>()) ? twiddle::maskUp<HI_BIT+1, T>() : 0; }
      static constexpr T bitsBelow() { return ((LO_BIT-1) >= 0) ? twiddle::maskDown<LO_BIT-1, T>() : 0; }

      constexpr T masked() const { return mask() & value; }

      constexpr T val() const { return std::is_signed<T>::value ? ((highBitExtended() ) >> LO_BIT) : (masked() >> LO_BIT); }
      constexpr T raw() const { return value; }

      constexpr explicit operator T() const { return val(); }

      template<typename R, int POS>
       constexpr R shiftedToPosMasked() const
        { return (POS <= LO_BIT ? (twiddle::mask<POS+HI_BIT-LO_BIT, POS, R>(static_cast<typename std::make_unsigned<R>::type>(value) >> twiddle::abs(LO_BIT-POS))) : (twiddle::mask<POS+HI_BIT-LO_BIT, POS, R>(static_cast<R>(value) << twiddle::abs(POS-LO_BIT)))); }
        template<typename R, int POS>
        constexpr R shiftedToPosNotMasked() const
        { return (POS <= LO_BIT ? (static_cast<typename std::make_unsigned<R>::type>(value) >> (LO_BIT-POS)) : (static_cast<R>(value) << (POS-LO_BIT))) ; }
      constexpr bool hiBitSet() const { return value & (1 << HI_BIT); }
//      constexpr T highBitExtended() const { return hiBitSet() ? (twiddle::mask<bitCount<T>()-1, HI_BIT, T>() | value) : (twiddle::mask<HI_BIT, 0, T>() & value); }
      constexpr T highBitExtended() const { return hiBitSet() ? (twiddle::mask<bitCount<T>()-1, HI_BIT, T>() | value) : value; }
//      template <typename R>
//      constexpr R get() const { return (bitCount<R>() <= numBits()) ? (static_cast<typename std::make_unsigned<R>::type>(value) >> LO_BIT) : (static_cast<typename std::make_unsigned<R>::type>(masked()) >> LO_BIT); }
//      constexpr T getRaw() const { return value; }
      std::string to_string() const {
            int numBits = bitCount<T>();
            std::string s = std::bitset<bitCount<T>()>(value).to_string();
            for (int i = HI_BIT+1; i <= numBits-1; i++) {
                size_t idx = static_cast<size_t>(numBits-1-i);
               s[idx] = (s[idx] == '1') ? '+' : '-';
            }
            for (int i = 0; i <= LO_BIT-1; i++) {
            size_t idx = static_cast<size_t>(numBits-1-i);
               s[idx] = (s[idx] == '1') ? '+' : '-';
            }
            return s;
        }
      std::string details() const {
          std::stringstream ss;
            int numBits = bitCount<T>();
            std::string s = std::bitset<bitCount<T>()>(value).to_string();
            for (int i = HI_BIT+1; i <= numBits-1; i++) {
                size_t idx = static_cast<size_t>(numBits-1-i);
               s[idx] = (s[idx] == '1') ? '+' : '-';
            }
            for (int i = 0; i <= LO_BIT-1; i++) {
            size_t idx = static_cast<size_t>(numBits-1-i);
               s[idx] = (s[idx] == '1') ? '+' : '-';
            }
            ss << s << " H: " << HI_BIT << " L: " << LO_BIT << " NumBits: " << (HI_BIT-LO_BIT+1);
            return ss.str();
        }
    };

    template <typename T>
    void show(T value)
    {
       std::cout << std::bitset<bitCount<T>()>(value) << std::endl;
    }

    template <int H, int L, typename T>
    void show(BitRange<H,L,T> value)
    {
       std::cout << value.to_string() << std::endl;
    }

    template <int H, int L, typename T>
    void details(BitRange<H,L,T> value)
    {
       std::cout << value.details() << std::endl;
    }


    template <typename T, int H1, int L1, typename T1, int H2, int L2, typename T2>
    constexpr auto concat(BitRange<H1,L1,T1> v1, BitRange<H2,L2,T2> v2)
    {
       return BitRange<H2-L2+H1-L1+1,0,T>(v1.template shiftedToPosMasked<T, H2-L2+1>() |
                                          v2.template shiftedToPosMasked<T, 0>(),
                                          BitRangeSrcType::isNormalizedAndMasked);
    }

    template <typename T, int H1, int L1, typename T1, int H2, int L2, typename T2, int H3, int L3, typename T3>
    constexpr auto concat(BitRange<H1,L1,T1> v1, BitRange<H2,L2,T2> v2, BitRange<H3,L3,T3> v3)
    {
       return BitRange<H3-L3+H2-L2+H1-L1+2,0,T>(v1.template shiftedToPosMasked<T, H2-L2+H3-L3+2>() |
                                                v2.template shiftedToPosMasked<T, H3-L3+1>() |
                                                v3.template shiftedToPosMasked<T, 0>(),
                                                BitRangeSrcType::isNormalizedAndMasked);
    }

    template <typename T, int H1, int L1, typename T1, int H2, int L2, typename T2, int H3, int L3, typename T3, int H4, int L4, typename T4>
    constexpr auto concat(const BitRange<H1,L1,T1> v1, BitRange<H2,L2,T2> v2, BitRange<H3,L3,T3> v3, BitRange<H4,L4,T4> v4)
    {
       return BitRange<H4-L4+H3-L3+H2-L2+H1-L1+3,0,T>(v1.template shiftedToPosMasked<T, H2-L2+H3-L3+H4-L4+3>() |
                                                      v2.template shiftedToPosMasked<T, H3-L3+H4-L4+2>() |
                                                      v3.template shiftedToPosMasked<T, H4-L4+1>() |
                                                      v4.template shiftedToPosMasked<T, 0>(),
                                                      BitRangeSrcType::isNormalizedAndMasked);
    }


    template <int H1, int L1, int H2, int L2, typename T>
    constexpr auto concat(BitRange<H1,L1,T> v1, BitRange<H2,L2,T> v2)
    {
        return concat<T, H1, L1, T, H2, L2, T>(v1, v2);
    }

    template <int H1, int L1, int H2, int L2, int H3, int L3, typename T>
    constexpr auto concat(BitRange<H1,L1,T> v1, BitRange<H2,L2,T> v2, BitRange<H3,L3,T> v3)
    {
        return concat<T, H1, L1, T, H2, L2, T, H3, L3, T>(v1, v2, v3);
    }

    template <int H1, int L1, int H2, int L2, int H3, int L3, int H4, int L4, typename T>
    constexpr auto concat(BitRange<H1,L1,T> v1, BitRange<H2,L2,T> v2, BitRange<H3,L3,T> v3, BitRange<H4,L4,T> v4)
    {
        return concat<H1, L1, H2, L2, H3, L3, H4, L4, T>(v1, v2, v3, v4);
    }

    template <int H1, int L1, int H2, int L2, typename T>
    constexpr auto concat(T a, T b)
    {
        BitRange<H1,L1,T> v1{a};
        BitRange<H2,L2,T> v2{b};
        return concat<H1, L1, H2, L2, T>(v1, v2);
    }

    template <int H1, int L1, int H2, int L2, int H3, int L3, typename T>
    constexpr auto concat(T a, T b, T c)
    {
        BitRange<H1,L1,T> v1{a};
        BitRange<H2,L2,T> v2{b};
        BitRange<H3,L3,T> v3{c};
        return concat<H1, L1, H2, L2, H3, L3, T>(v1, v2, v3);
    }

    template <int H1, int L1, int H2, int L2, int H3, int L3, int H4, int L4, typename T>
    constexpr auto concat(T a, T b, T c, T d)
    {
        BitRange<H1,L1,T> v1{a};
        BitRange<H2,L2,T> v2{b};
        BitRange<H3,L3,T> v3{c};
        BitRange<H4,L4,T> v4{d};
        return concat<T, H1, L1, T, H2, L2, T, H3, L3, T, H4, L4, T>(v1, v2, v3, v4);
    }

    template <typename T, int H1, int L1, typename T1, int H2, int L2, typename T2>
    constexpr auto prependBits(T1 v1, BitRange<H2,L2,T2> v2, BitRangeSrcType srcType = BitRangeSrcType::notNormalizedNotMasked)
    {
       return concat<T>(BitRange<H1,L1,T1>{v1, srcType}, v2);
    }

    template <typename T, int H1, int L1, typename T1, int H2, int L2, typename T2>
    constexpr auto appendBits(BitRange<H2,L2,T2> v2, T1 v1, BitRangeSrcType srcType = BitRangeSrcType::notNormalizedNotMasked)
    {
       return concat<T>(v2, BitRange<H1,L1,T1>{v1, srcType});
    }


    template <int H, int L, typename T>
    constexpr auto bits(T v, BitRangeSrcType srcType = BitRangeSrcType::notNormalizedNotMasked)
    {
        return BitRange<H,L,T>{v, srcType};
    }
}

#endif // BITRANGE_H
