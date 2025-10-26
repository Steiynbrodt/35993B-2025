#ifndef GRID_UTILS_MINMAXSWAP
#define GRID_UTILS_MINMAXSWAP

template<typename T>
inline void mySwap(T& a, T& b) {
    T tmp = a;
    a = b;
    b = tmp;
}

template<typename T>
inline const T& myMin(const T& a, const T& b) {
    return (b < a) ? b : a;
}

template<typename T>
inline const T& myMax(const T& a, const T& b) {
    return (a < b) ? b : a;
}
#endif