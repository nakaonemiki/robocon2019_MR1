#include "Arduino.h"

// 制御周期
#define INT_TIME			( 0.01 )

// 自己位置推定用エンコーダ関連
#define MEASURE_HANKEI		( 19.05 )	// エンコーダの半径
#define HANKEI_D			( 190.0 )	// 二つ平行についているエンコーダとロボットの中心の距離
#define HANKEI_L			( 249.5 )	// 上のエンコーダに対して垂直についているエンコーダとロボットの中心の距離
#define MEASURE_RES_MUL_X	( 800 )	// エンコーダの分解能(200)と4逓倍をかけたもの
#define MEASURE_RES_MUL_Y	( 400 )	// エンコーダの分解能(100)と4逓倍をかけたもの

