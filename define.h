#include "Arduino.h"

// 制御周期
#define INT_TIME			( 0.01 )//( 0.001 )

// フェーズ管理
#define STATE1      ( 10 )// スタートからゲルゲ受け渡しまで
#define STATE2      ( 1 )// ゲルゲ受け渡し後からシャガイ取得まで
#define STATE3      ( 1 )// シャガイ取得後からスローイングゾーン待機まで
#define STATE4      ( 1 )// スローイングゾーンでの走行
#define STATE_ALL   ( STATE1 + STATE2 + STATE3 + STATE4 )

// VL53L0X
#define SENSOR_NUM  4 // 使用するセンサーの数
#define ADDRESS_DEFALUT 0b0101001 // 0x29
#define ADDRESS_00 (ADDRESS_DEFALUT + 2)

// 自己位置推定用エンコーダ関連
#define MEASURE_HANKEI		( 0.01905 )//( 0.01905 )//( 19.05 )	// エンコーダの半径
#define MEASURE_HANKEI_X    ( 0.0188618 )
#define MEASURE_HANKEI_X_R  ( 0.0188618 )//( 0.018952 )//( 0.019392 )
#define MEASURE_HANKEI_X_L  ( 0.01900344 )//( 0.019002 )//( 0.019325 )
#define MEASURE_HANKEI_Y    ( 0.01900344 )//( 0.019032 )//( 0.019609 )//( 19.05 )

#define MEASURE_HANKEI_D	( 0.190585 )//( 0.19075 )//( 0.19 )	// 二つ平行についているエンコーダとロボットの中心の距離
#define MEASURE_HANKEI_L	( 0.251007 )//( 0.25 )//( 0.2495 )	// 上のエンコーダに対して垂直についているエンコーダとロボットの中心の距離
#define MEASURE_RES_MUL_X	( 800 )//( 800 )	// エンコーダの分解能(200)と4逓倍をかけたもの
#define MEASURE_RES_MUL_Y	( 800 )	// エンコーダの分解能(100)と4逓倍をかけたもの

// メカナム関連
#define MECANUM_RES			( 500 )
#define MECANUM_HANKEI		( 0.05 )
#define MECANUM_HANKEI_D	( 0.15561 )
#define MECANUM_HANKEI_L	( 0.26023 )

// RoboClaw関連
#define ADR_MD1             ( 128 )
#define ADR_MD2             ( 129 )

const double _2PI_MEASRMX = 2.0 * PI / MEASURE_RES_MUL_X;
const double _2PI_MEASRMY = 2.0 * PI / MEASURE_RES_MUL_Y;
const double _0P5_MEASHD = 0.5 / MEASURE_HANKEI_D;
const double _MECAHD_ADD_MECAHL = MECANUM_HANKEI_D + MECANUM_HANKEI_L;
const double _2MECAR_PI = 2.0 * MECANUM_RES / PI;