#ifndef _HC220CAN_H_
#define _HC220CAN_H_
#pragma pack(push, 1)
struct Msg387{
    bool flag = false;
    int RawIMU_VehicleGpsWeek;//: 2249 week,
    double RawIMU_VehicleGpsTime;//: 354086.15 s,
    int RawIMU_VehicleReceived;//: 0,
    int RawIMU_VehicleE2eCounter;//: 13,
    int RawIMU_VehicleE2eCRC;//: 128,
    double RawIMU_VehicleGyroX;//: -0.08799999999996544 deg/s,
    double RawIMU_VehicleGyroY;//: 0.03700000000003456 deg/s,
    double RawIMU_VehicleGyroZ;//: -0.024999999999977263 deg/s,
    double RawIMU_VehicleAccelX;//: 0.09520000000000017 g,
    double RawIMU_VehicleAccelY;//: -0.014279999999999404 g,
    double RawIMU_VehicleAccelZ;//: 0.9989800000000013 g,
    double RawIMU_VehicleTemp;//: 31.23 ¡æ,
    double RawIMU_VehicleErrStatus_gyro;//: 0,
    double RawIMU_VehicleErrStatus_gyrox;//: 0,
    double RawIMU_VehicleErrStatus_gyroy;//: 0,
    double RawIMU_VehicleErrStatus_gyroz;//: 0,
    double RawIMU_VehicleErrStatus_acc;//: 0,
    double RawIMU_VehicleErrStatus_accx;//: 0,
    double RawIMU_VehicleErrStatus_accy;//: 0,
    double RawIMU_VehicleErrStatus_accz;//: 0,
    double RawIMU_Vehicle_Yaw;//: 3.2800000000000002
};
//can0  388  [32]  08 C9 15 1A E8 A6 0D 8C 2D 2A D1 0D B7 05 A2 34 59 4C EE 00 14 91 E3 17 56 A4 60 02 D9 03 00 00 ::
struct Msg388{
    bool flag = false;
    int INS_GpsWeek;//: 2249 week,
    double INS_GpsTime;//: 354085.03 s,
    double INS_Received;//: 0,
    int INS_E2eCounter;//: 13,
    int INS_E2eCRC;//: 140,
    double INS_PosLat;//: 31.244919949999996 deg,
    double INS_PosLon;//: 121.59287384999999 deg,
    double INS_PosAlt;//: 41.66399999999976 m,
    double INS_StdLat;//: 3.29 m,
    double INS_StdLon;//: 4.83 m,
    double INS_StdAlt;//: 3.73 m,
    double INS_Heading;//: 272.06 deg,
    double INS_Speed;//: 0.11 m/s,
    int INS_StatPos;//: 6,
    int INS_StatIns;//: 1,
    int INS_SensorUsed;//: 3
};

//can0  389  [32]  08 C9 15 1A E8 A6 0D 66 FF EA 00 00 FF F6 FD DE FF AD 45 81 01 E0 1E 01 E0 01 00 10 01 64 00 00 ::
struct Msg389{
    bool flag = false;
    int INS_GpsWeek;//: 2249 week,
    double INS_GpsTime;//: 354085.03 s,
    double INS_Received;//: 0,
    int INS_E2eCounter;//: 13,
    int INS_E2eCRC;//: 102,
    double INS_VelE;//: -0.11 m/s,
    double INS_VelN;//: 0.0 m/s,
    double INS_VelU;//: -0.05 m/s,
    double INS_Pitch;//: -5.46 deg,
    double INS_Roll;//: -0.8300000000000001 deg,
    double INS_Yaw;//: 177.93 deg,
    double INS_StdVelE;//: 0.3 m/s,
    double INS_StdVelN;//: 0.3 m/s,
    double INS_StdVelU;//: 0.3 m/s,
    double INS_StdPitch;//: 0.01 deg,
    double INS_StdRoll;//: 0.01 deg,
    double INS_StdYaw;//: 0.01 deg,
    int INS_StatPos;//: 6,
    int INS_StatIns;//: 1
};

//can0  38D  [32]  08 C9 15 1A E8 A6 0D 20 7A 12 07 A1 20 7A 12 07 A1 20 7A 12 07 A1 20 7A 12 07 A1 20 7A 12 00 00 ::
struct Msg38d{
    bool flag = false;
    int INS_GpsWeek;//: 2249 week,
    double INS_GpsTime;//: 354085.03 s,
    double INS_Received;//: 0,
    int INS_E2eCounter;//: 13,
    int INS_E2eCRC;//: 32,
    double INS_VehicleGyroX;//: 0.0 deg/s,
    double INS_VehicleGyroY;//: 0.0 deg/s,
    double INS_VehicleGyroZ;//: 0.0 deg/s,
    double INS_VehicleAccelX;//: 0.0 g,
    double INS_VehicleAccelY;//: 0.0 g,
    double INS_VehicleAccelZ;//: 0.0 g,
    double INS_VehicleAccelCarX;//: 0.0 g,
    double INS_VehicleAccelCarY;//: 0.0 g,
    double INS_VehicleAccelCarZ;//: 0.0 g
};

//0x382 
struct Msg382{
    bool flag = false;
    int MON_TIME_GPS_week;//: 2249 week,
    double MON_GPS_second;//: 354086.0 s,
    int MON_Received;//: 0,
    int MON_E2eCounter;//: 4,
    int MON_E2eCRC;//: 197,
    double MON_gyo_bias_x;//: 0.0 deg/s,
    double MON_gyo_bias_y;//: 0.0 deg/s,
    double MON_gyo_bias_z;//: 0.0 deg/s,
    double MON_gyo_sf_x;//: 0.0 %,
    double MON_gyo_sf_y;//: 0.0 %,
    double MON_gyo_sf_z;//: 0.0 %,
    double MON_acc_bias_x;//: 0.0 mg,
    double MON_acc_bias_y;//: 0.0 mg,
    double MON_acc_bias_z;//: 0.0 mg,
    double MON_acc_sf_x;//: 0.0 %,
    double MON_acc_sf_y;//: 0.0 %,
    double MON_acc_sf_z;//: 0.0 %
};

//msg381
struct Msg381{
    bool flag = false;
    int MON_TIME_GPS_week;// 2249 week,
    double MON_GPS_second;// 354086.0 s,
    int MON_E2eCounter;// 4,
    int MON_Received;// 0,
    int MON_E2eCRC;// 57,
    int MON_cpu_usage;// 28 %,
    int MON_mem_usage;// 0 %,
    int MON_cal_stat;// 0,
    int MON_imu_use_flag;// 1,
    int MON_gnss_use_flag;// 1,
    int MON_odo_use_flag;// 0,
    int MON_rtk_stat;// 6,
    int MON_cal_rate;// 0 %,
    int MON_ins_stat;// 1,
    double MON_vel;// 0.0 km/h,
    double MON_odo_sf;// 0.0 %,
    double MON_install_imu2car_roll;// 0.0 deg,
    double MON_install_imu2car_pitch;// 0.0 deg,
    double MON_install_imu2car_yaw;// 0.0 deg,
    int MON_frq_use_stat;// 237,
    int MON_CN_BD_L5;// 0 dB,
    int MON_CN_BD_L1;// 0 dB,
    int MON_CN_GPS_L1;// 0 dB,
    int MON_CN_GPS_L5;// 0 dB,
    int MON_CN_GAL_L5;// 0 dB,
    int MON_CN_GAL_L1;// 0 dB,
    int MON_NumStas_BD_L1;// 0 ¿Å,
    int MON_NumStas_GPS_L1;// 0 ¿Å,
    int MON_NumStas_BD_L5;// 0 ¿Å,
    int MON_NumStas_GPS_L5;// 0 ¿Å,
    int MON_NumStas_GAL_L1;// 0 ¿Å
    int MON_NumStas_GAL_L5;// 0 ¿Å,
};

//msg383
struct Msg383{
    bool flag = false;
    int RawGnss_GpsWeek;//: 2281 week,
    double RawGnss_GpsTime;//: 266831.0 s,
    int RawGnss_Received;//: 0,
    int RawGnss_E2eCounter;//: 9,
    int RawGnss_E2eCRC;//: 118,
    double RawGnss_VelE;//: -0.02 m/s,
    double RawGnss_VelN;//: -0.025 m/s,
    double RawGnss_VelU;//: -0.02 m/s,
    double RawGnss_StdVelE;//: 1.35 m/s,
    double RawGnss_StdVelN;//: 1.21 m/s,
    double RawGnss_StdVelU;//: 1.4000000000000001 m/s,
    double RawGnss_Speed;//: 0.03 m/s,
    double RawGnss_Heading;//: 221.07 deg,
    double RawGnss_Hdop;//: 0.9,
    double RawGnss_Pdop;//: 2.3000000000000003,
    double RawGnss_Vdop;//: 2.2,
    double RawGnss_Tdop;//: 1.6,
    double RawGnss_Gdop;//: 2.8000000000000003,
    int RawGnss_Leaps;//: 18 s,
    int RawGnss_NumSats_used;//: 0
};

//msg385
struct Msg385{
    bool flag = false;
    int RawGnss_GpsWeek;//: 2281 week,
    double RawGnss_GpsTime;//: 266828.4 s,
    int RawGnss_Received;//: 0,
    int RawGnss_E2eCounter;//: 12,
    int RawGnss_E2eCRC;//: 65,
    double RawGnss_PosLat;//: 31.207821530000004 deg,
    double RawGnss_PosLon;//: 121.59460402000002 deg,
    double RawGnss_PosAlt;//: 16.89300000000003 m,
    double RawGnss_Undulation;//: 11.5 m,
    double RawGnss_StdLat;//: 0.009000000000000001 m,
    double RawGnss_StdLon;//: 0.007 m,
    double RawGnss_StdAlt;//: 0.022 m,
    double RawGnss_Age;//: 1.4000000000000001 s,
    int RawGnss_PosType;//: 4,
    int RawGnss_NumSats;//: 29
};

struct HCCGI220_
{

    int stat;//  RTKLIB的解类型
    Msg381 msg381;    //0x381 1HZ
    Msg383 msg383;    //0x383 5HZ
    Msg385 msg385;    //0x385 5HZ
    Msg387 msg387;    //0x386 0x387 100hz
    Msg388 msg388;    //0x388 100hz
    Msg389 msg389;    //0x389 100hz
    Msg38d msg38d;    //0x38D 100hz
    Msg382 msg382;    //0x382 1hz
};
#pragma pack(pop)
// void HCCGI220Tosol(const sol_t sol,)

#endif