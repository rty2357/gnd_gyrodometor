/**
 * @file gnd/gnd_gyrodometry.hpp
 *
 * @brief Template class for gnd_gyro_odometry.
 **/
#ifndef gnd_gyrodometry_HPP
#define gnd_gyrodometry_HPP

#ifdef WIN32
  #ifdef gnd_gyrodometry_EXPORTS
    #define gnd_gyrodometry_API __declspec(dllexport)
  #else
    #define gnd_gyrodometry_API __declspec(dllimport)
  #endif
#else
  #define gnd_gyrodometry_API
#endif

#endif // gnd_gyro_odometry_HPP
