#include "Arduino.h"

// 制御周期
#define INT_TIME			( 0.01 )

// 自己位置推定用エンコーダ関連
#define MEASURE_HANKEI		( 0.01905 )//( 19.05 )	// エンコーダの半径
#define MEASURE_HANKEI_X_L  ( 0.019392 )
#define MEASURE_HANKEI_X_R  ( 0.019325 )
#define MEASURE_HANKEI_Y    ( 0.019609 )

#define MEASURE_HANKEI_D	( 0.192236 )//( 0.19 )//( 190.0 )	// 二つ平行についているエンコーダとロボットの中心の距離
#define MEASURE_HANKEI_L	( 0.250101 )//( 0.2495 )//( 249.5 )	// 上のエンコーダに対して垂直についているエンコーダとロボットの中心の距離
#define MEASURE_RES_MUL_X	( 800 )	// エンコーダの分解能(200)と4逓倍をかけたもの
#define MEASURE_RES_MUL_Y	( 400 )	// エンコーダの分解能(100)と4逓倍をかけたもの

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