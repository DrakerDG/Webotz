#ifndef CONFIG_HPP
#define CONFIG_HPP

const int    N_SEN =    8;     // Number of sensors
const int    THOLD =  500;     // IR sensor threshold     (500)
const int    NR_GS = 1000;     // Normal max value of sensors
const int  C_THOLD =   20;     // Curvature threshold      (20)  (10)
const int STOP_DLY =  300;     // Pause lap time          (300)
const int  BRAKE_M =   20;     // Anticipation of braking  (20)  (40)
const double  FRAC =  0.9;     // Fraction of segment     (0.9) (0.4)  

const int    SP000 =  20;      //  20 rad/s  ≈   190 RPM  ≈  0.34  m/s  Calibration
const int    SP001 =  60;      //  60 rad/s  ≈   572 RPM  ≈  1.02  m/s  Initial Run
const int    SP002 =  90;      //  90 rad/s  ≈   859 RPM  ≈  1.53  m/s  Limit speed low
const int    SP003 =  90;      //  90 rad/s  ≈   859 RPM  ≈  1.53  m/s  Optimized Run     (90)  (150)
const int    SP004 = 100;      // 100 rad/s  ≈   954 RPM  ≈  1.70  m/s  Limit speed hi   (100)  (190)
const int    SP005 = 220;      // 220 rad/s  ≈  2100 RPM  ≈  3.74  m/s  <--- Unused
const int    SP006 = 240;      // 240 rad/s  ≈  2291 RPM  ≈  4.08  m/s  <--- Unused
const int    ST_CH =   1;      //   1 rad/s  ≈     9 RPM  ≈  0.017 m/s  Incremental and decremental speed

#endif
