/************************************************/
// 
// 長さの単位はmとする
// 
/************************************************/

#include <Arduino.h>
#include <RoboClaw.h>
#include "define.h"
#include "MsTimerTPU3.h"
#include "phaseCounter.h"
#include "PIDclass.h"
#include "Filter.h"
#include "lpms_me1.h"
//#include "VL53L0X.h"
#include "Wire.h"
#include "reboot.h"
#include "SDclass.h"
#include "MotionGenerator.h"

#define PIN_RED		(44)
#define PIN_BLUE	(45)
#define PIN_BUTTON1 (49)
#define PIN_BUTTON2 (50)

#define PIN_BUTTON_NORMAL (46)
#define PIN_BUTTON_RETRY1 (48)
#define PIN_BUTTON_RETRY2 (47)
//#define PIN_EXBUTTON1 (45)

#define PIN_EXLED1 (25)
//#define PIN_EXLED2 (27)

// 自己位置推定用のエンコーダ
phaseCounter Enc1(1);
phaseCounter Enc2(2);
phaseCounter Enc3(3);

// RoboClaw
RoboClaw MD(&Serial2,1);//10);

// lpms-me1
lpms_me1 lpms(&Serial3);

boolean pidPreError_update = false;

mySDclass mySD;

MotionGenerator motion(FOLLOW_TANGENT); // 経路追従(接線方向向く)モードで初期化

// VL53L0X
/*const int VL53L0X_GPIO[SENSOR_NUM] = {A0, A1, A2, A3};
VL53L0X gSensor[SENSOR_NUM]; // 使用するセンサークラス配列
unsigned int sensVal[SENSOR_NUM] = {0};*/

int EncountA = 0, EncountB = 0, EncountC = 0;
int pre_EncountA = 0, pre_EncountB = 0, preAngleC = 0;
int pre_tmpEncA = 0, pre_tmpEncB = 0, pre_tmpEncC = 0;
int ledcount = 0;
boolean stopRoller = false;
int swTiming = 0;

int count_10ms = 0;//0;
boolean flag_10ms = false;
boolean flag_20ms = false;
boolean flag_5s = false;

int zone; // 赤か青か
int retry_num = 0; // リトライモード 0:通常，1:リトライ1，2:リトライ2

// phase で動作フェーズを管理
int phase = 0;
// 0 : スタートゾーンで待機するフェーズ（指令値一定）
//    【フェーズ移行条件】：スイッチが押されるまで
// 1 : ベジエ曲線を，ロボットの角度が接線方向になるように追従するフェーズ（軌道追従制御）
//    【フェーズ移行条件】：path_num が所定の数値になったら
// 2 : ゲルゲ受け渡し後に，じわじわ動いて壁に接触するフェーズ（指令値一定）
//    【フェーズ移行条件】：すべてのリミットスイッチが押された状態になったら
// 3 : 壁から離れるフェーズ（位置制御）
//    【フェーズ移行条件】：目標位置と角度に収束したら
// 4 : 角度だけ変えるフェーズ（位置制御）
//    【フェーズ移行条件】：目標位置と角度に収束したら
// 5 : シャガイの前に移動するフェーズ（軌道追従制御）
//    【フェーズ移行条件】：path_num が所定の数値になったら
// 6 : 所定の位置(シャガイ前)にとどまるフェーズ（位置制御）
//    【フェーズ移行条件】：シャガイハンド閉じて，持ち上げたら / スイッチを押されたら
// 7 : シャガイを取ったあと，スローイングゾーンの前に移動するフェーズ（軌道追従制御）
//    【フェーズ移行条件】：path_num が所定の数値になったら
// 8 : スローイングゾーンの前でとどまるフェーズ（位置制御）
//    【フェーズ移行条件】：スイッチを押されたら
// 9 : シャガイを投げる位置まで移動するフェーズ（軌道追従制御）
//    【フェーズ移行条件】：path_num が所定の数値になったら
// 10 : シャガイを投げる位置で待機するフェーズ（位置制御）
//    【フェーズ移行条件】：スイッチを押されたら
//(11): 2個目のシャガイの前まで移動するフェーズ（軌道追従制御）
//    【フェーズ移行条件】：シャガイハンド閉じて，持ち上げたら / スイッチを押されたら
//(12): 所定の位置(シャガイ前)にとどまるフェーズ（位置制御）
//    【フェーズ移行条件】：シャガイハンド閉じて，持ち上げたら / スイッチを押されたら
//(13): シャガイを投げる位置まで移動するフェーズ（軌道追従制御）
//    【フェーズ移行条件】：path_num が所定の数値になったら
//(14): シャガイを投げる位置で待機するフェーズ（位置制御）
//    【フェーズ移行条件】：スイッチを押されたら
//(15): 3個目のシャガイの前まで移動するフェーズ（軌道追従制御）
//    【フェーズ移行条件】：シャガイハンド閉じて，持ち上げたら / スイッチを押されたら
//(16): 所定の位置(シャガイ前)にとどまるフェーズ（位置制御）
//    【フェーズ移行条件】：シャガイハンド閉じて，持ち上げたら / スイッチを押されたら
//(17): シャガイを投げる位置まで移動するフェーズ（軌道追従制御）
//    【フェーズ移行条件】：path_num が所定の数値になったら
//(18): シャガイを投げる位置で待機するフェーズ（位置制御）
//    【フェーズ移行条件】：スイッチを押されたら
//(100): リトライ（スタート待機）
//    【フェーズ移行条件】：スイッチが押されたら phase -> 101 へ
//(101): リトライ（直進後にじわじわ動いて位置補正-->シャガイ取得・投擲）
//    【フェーズ移行条件】：一定距離進んだら phae -> 2 へ

int data1[ 1300 ];//[ 12000 ];
int data2[ 1300 ];
int data3[ 1300 ];
int data4[ 1300 ];
int data5[ 1300 ];
int data6[ 1300 ];
int data7[ 1300 ];

int *pdata1 = data1;
int *pdata2 = data2;
int *pdata3 = data3;
int *pdata4 = data4;
int *pdata5 = data5;
int *pdata6 = data6;
int *pdata7 = data7;

char cmd = 0b00000000;

/* double Kakudoxl, Kakudoxr, Kakudoy, tmpKakudoy;
double Posix, Posiy, Posiz; */
double tmpPosix = 0.0, tmpPosiy = 0.0, tmpPosiz = 0.0;


// グローバル変数の設定
double gPosix, gPosiy, gPosiz;//1.5708;//0;
double angle_rad;
const double _ANGLE_DEG = 45.0;

// 最大最小範囲に収まるようにする関数
double min_max(double value, double minmax)
{
    if(value > minmax) value = minmax;
    else if(value < -minmax) value = -minmax;
    return value;
}

void timer_warikomi(){
	static int count = 0;
	static double pre_EncountA = 0.0, pre_EncountB = 0.0, pre_EncountC = 0.0;
    static double preAngleA = 0.0, preAngleB = 0.0, preAngleC = 0.0;
    static double preVxl = 0.0, preVyl = 0.0, preVzl = 0.0;
    count++;

	// R1370
	double rawangle, diff_angle;
	static double pre_rawangle = 0.0, angle_deg = _ANGLE_DEG;//0.0;
	boolean recv_done = false;
	
	if(ledcount >= 25) {
		if(digitalRead(PIN_LED0) == LOW){
			digitalWrite(PIN_LED0, HIGH);
			if(phase == 100) digitalWrite(PIN_EXLED1, HIGH);
		} else {
			digitalWrite(PIN_LED0, LOW);
			if(phase == 100) digitalWrite(PIN_EXLED1, LOW);
		}
		ledcount = 0;
	}
	ledcount++;

    // 自己位置推定用エンコーダのカウント値取得
    EncountA = -Enc1.getCount();// Enc1.getCount();//-Enc1.getCount();//-Enc1.getCount();	// MTU1, X//xl
    EncountB =  Enc2.getCount();//-Enc2.getCount();//-Enc2.getCount();	// MTU2, //y
    EncountC =  Enc3.getCount();//-Enc3.getCount();// Enc3.getCount();// Enc3.getCount();	// TPU1, Y//xr
	
	// 角度   encountはdoubleに型変換した方がいいかもしれない
	double Kakudoxl, Kakudoxr, Kakudox, Kakudoy;
	Kakudoxl = ( double )( EncountC - pre_EncountC ) * _2PI_MEASRMX;//( double )( EncountA - pre_EncountA ) * _2PI_MEASRMX;
	Kakudoxr = ( double )( EncountA - pre_EncountA ) * _2PI_MEASRMX;//( double )( EncountC - pre_EncountC ) * _2PI_MEASRMX;
	Kakudox = ( double )( EncountA - pre_EncountA ) * _2PI_MEASRMX;
	Kakudoy = ( double )( EncountC - pre_EncountC ) * _2PI_MEASRMY;//( EncountB - pre_EncountB ) * _2PI_MEASRMY;

	angle_rad = (double)lpms.get_z_angle();

	// tmpKakudoy += Kakudoy;

	// ローカル用(zは角度)
	double Posix, Posiy, Posiz;
	static double pre_angle_rad = angle_rad;
	double angle_diff;
	angle_diff = angle_rad - pre_angle_rad;
	Posiz = angle_diff;//( MEASURE_HANKEI_X_R * Kakudoxr - MEASURE_HANKEI_X_L * Kakudoxl ) * _0P5_MEASHD;//
	Posix = MEASURE_HANKEI_X * Kakudox;//( MEASURE_HANKEI_X_L * Kakudoxl + MEASURE_HANKEI_X_R * Kakudoxr ) * 0.5;
	Posiy = MEASURE_HANKEI_Y * Kakudoy;// + MEASURE_HANKEI_L * Posiz;//MEASURE_HANKEI_Y * Kakudoy - MEASURE_HANKEI_L * Posiz;
	
	//static double tmpPosix = 0.0, tmpPosixl = 0.0, tmpPosixr = 0.0, tmpPosiy = 0.0, tmpPosiz = 0.0;
	tmpPosix += Posix;
	tmpPosiy += Posiy;
	tmpPosiz += Posiz;

	// 回転中心の座標
	/* double Posixrc, Posiyrc;
	double _Posix, _Posiy;
	if( fabs(Posixr - Posixl) > 0.0001){//0.001 ){	
		Posixrc = ( Posiy  * 2.0 * MEASURE_HANKEI_D) / ( Posixr - Posixl ) + MEASURE_HANKEI_L;
		Posiyrc = ( Posixl * 2.0 * MEASURE_HANKEI_D) / ( Posixr - Posixl ) + MEASURE_HANKEI_D;
	
		// 回転中心とロボットの中心までの距離(ロー)
		double rho = sqrt(pow(Posixrc, 2.0) + pow(Posiyrc, 2.0));
		// Posixrc, Posiyrcを回転中心として，前の座標から今の座標までの円周
		double deltaL = rho * Posiz;

		// グローバル用(zは角度)
		
		//gPosix += Posix * cos( gPosiz ) - Posiy * sin( gPosiz );//Posix * cos( tmp_Posiz ) - Posiy * sin( tmp_Posiz );
		//gPosiy += Posix * sin( gPosiz ) + Posiy * cos( gPosiz );//Posix * sin( tmp_Posiz ) + Posiy * cos( tmp_Posiz );
		//gPosiz += Posiz;
		_Posix = deltaL * Posiyrc;
		_Posiy = deltaL * Posixrc;
		//gPosiz += Posiz;
	}else{
		_Posix = Posix;
		_Posiy = Posiy;
	} */
	
	double tmp_Posiz = gPosiz + ( Posiz * 0.5 ); // つまりgPosi + ( Posiz / 2.0 );
	gPosiz += Posiz;
	gPosix += Posix * cos( gPosiz ) - Posiy * sin( gPosiz );//Posix * cos( tmp_Posiz ) - Posiy * sin( tmp_Posiz );
	gPosiy += Posix * sin( gPosiz ) + Posiy * cos( gPosiz );//Posix * sin( tmp_Posiz ) + Posiy * cos( tmp_Posiz );
	//gPosiz += Posiz;

	static int count_5s = 0;
	count_5s++;
	if(count_5s == 500){
		flag_5s = true;
		//phase = 1;
	}
	//if( flag_5s ){
		count_10ms++;
		if( count_10ms == 1 ){
			flag_10ms = true;
			count_10ms = 0;
		}
	//}
	static int count_20ms = 0;
	count_20ms++;
	if( count_20ms == 2 ){
		flag_20ms = true;
		count_20ms = 0;
	}

	pre_EncountA = EncountA;
	pre_EncountB = EncountB;
	pre_EncountC = EncountC;

	pre_angle_rad = angle_rad;

	// pre_tmpEncA = tmpEncA;
	// pre_tmpEncB = tmpEncB;
	// pre_tmpEncC = tmpEncC;
	
	/*Serial.print( onx, 2 );
	Serial.print( "\t" );
	Serial.print( ony, 2 );
	Serial.print( "\t" );
	Serial.print( gPosix, 2 );
	Serial.print( "\t" );
	Serial.print( gPosiy, 2 );
	Serial.print( "\t" );
	Serial.println(phase);//( gPosiz, 2 );*/
	
}


void setup() {
	pinMode(PIN_SW, INPUT);		// reboot用
	pinMode(PIN_LED0, OUTPUT);
    pinMode(PIN_LED1, OUTPUT);
	pinMode(PIN_LED2, OUTPUT);
    pinMode(PIN_LED3, OUTPUT);
	
	// SW
	pinMode(A0, INPUT);
	pinMode(A1, INPUT);
	pinMode(A2, INPUT);
	pinMode(A3, INPUT);
	pinMode(A4, INPUT);
	pinMode(A5, INPUT);

	pinMode(PIN_BUTTON_NORMAL, INPUT);
	pinMode(PIN_BUTTON_RETRY1, INPUT);
	pinMode(PIN_BUTTON_RETRY2, INPUT);
	//pinMode(PIN_EXBUTTON1, INPUT);

	pinMode(PIN_EXLED1, OUTPUT);
	//pinMode(PIN_EXLED2, OUTPUT);

	pinMode(49, INPUT);
	pinMode(50, INPUT);
	pinMode(51, INPUT);
	pinMode(PIN_RED, OUTPUT);
	pinMode(PIN_BLUE, OUTPUT);

	// RoboClaw
	MD.begin(115200);

	// PCと通信
	Serial.begin(230400);
	// mbedと通信
	Serial1.begin(115200);
	
	if(lpms.init() == 0){
		digitalWrite(PIN_LED3, HIGH);
	}

	// VL53L0X
	/* Wire.begin();
	for (int i = 0; i < SENSOR_NUM; i++){
		pinMode(VL53L0X_GPIO[i], OUTPUT);
		digitalWrite(VL53L0X_GPIO[i], LOW);
	}

	for (int i = 0; i < SENSOR_NUM; i++) {
		// センサを初期化
		pinMode(VL53L0X_GPIO[i], INPUT);
		if (gSensor[i].init() == true){
			gSensor[i].setTimeout(50);//(100);
			gSensor[i].startContinuous();
			int address = ADDRESS_00 + (i * 2);
			gSensor[i].setAddress(address);
			//gSensor[i].setMeasurementTimingBudget(20000);
		}else{
			Serial.print("Sensor ");
			Serial.print(i);
			Serial.println(" error");
		}
	} */

	/*** SDカード利用のために追加　2019/05/05 ***/
	Serial.print("Initializing ...");
	mySD.init();
	Serial.print("Path reading ...");
	
	// cmd = BIT_START;
	// Serial1.print('L');
	// Serial1.print(cmd); // 初期化
	// Serial1.print('\n');

	int button_state;
	do{
		button_state = 0;
		button_state |= !digitalRead(PIN_BUTTON_NORMAL); // 「!」で反転して格納
		button_state |= !digitalRead(PIN_BUTTON_RETRY1)<<1;
		button_state |= !digitalRead(PIN_BUTTON_RETRY2)<<2;
	}while( button_state == 0x00 ); // どのボタンも押されていない限り続ける

	int actpathnum;
	if( !digitalRead(51) ){	// 赤
		cmd = BIT_RED;
		Serial1.print('L');
		Serial1.print(cmd); // 初期化
		Serial1.print('\n');

		digitalWrite(PIN_RED, HIGH);
		actpathnum = mySD.path_read(RED, motion.Px, motion.Py, motion.refvel, motion.refangle, motion.acc_mode, motion.acc_count, motion.dec_tbe);
		Serial.println(actpathnum);
		//mySD.path_read(RED, Px_SD, Py_SD, refvel_SD, refangle_SD);

		if((button_state & 0x01)){
			// 通常スタート
			Serial.println("Normal Start at RED zone");
			retry_num = 10;
			phase = 0;
		}else if((button_state & 0x02)){
			// リトライ1
			Serial.println("Re-start at RED zone");
			retry_num = 11;
			phase = 100;
		}/*else if(button_state & 0x04 == 0x04){
			// リトライ2
			retry_num = 12;
			gPosiz = 3.14159265;
			phase = 100;
		}*/

		zone = RED;
	}else{					// 青
		cmd = BIT_BLUE;
		Serial1.print('L');
		Serial1.print(cmd); // 初期化
		Serial1.print('\n');

		digitalWrite(PIN_BLUE, HIGH);
		actpathnum = mySD.path_read(BLUE, motion.Px, motion.Py, motion.refvel, motion.refangle, motion.acc_mode, motion.acc_count, motion.dec_tbe);
		Serial.print("path num: ");
		Serial.println(actpathnum);
		//mySD.path_read(BLUE, Px_SD, Py_SD, refvel_SD, refangle_SD);

		if(button_state & 0x01){
			// 通常スタート
			Serial.println("Normal Start at BLUE zone");
			retry_num = 0;
			phase = 0;
		}else if(button_state & 0x02){
			// リトライ1
			Serial.println("Re-start at BLUE zone");
			retry_num = 1;
			phase = 100;
		}/*else if(button_state & 0x04 == 0x04){
			// リトライ2
			retry_num = 2;
			gPosiz = 0.0;
			phase = 100;
		}*/

		zone = BLUE;
	}
	if(actpathnum >= 0){
		digitalWrite(PIN_EXLED1, HIGH); // 読み込みが完了したらLED点灯
	}

	gPosix = motion.Px[0];
	gPosiy = motion.Py[0];


	Serial.println(motion.Px[0]);
	mySD.make_logfile();
	/*** SDカード利用のために追加　2019/05/05 ***/

	motion.initSettings(); // これをやっていないと足回りの指令速度生成しない
	motion.setConvPara(0.02, 0.997); // 初期化
	motion.setMaxPathnum(actpathnum); // パス数の最大値

	//delay(5000);
	// タイマー割り込み(とりあえず10ms)
	MsTimerTPU3::set((int)(INT_TIME * 1000), timer_warikomi); // 10ms period
	MsTimerTPU3::start();
	
	// 自己位置推定用のエンコーダ
	Enc1.init();
	Enc2.init();
	Enc3.init();
}


// reboot用関数
/* void reboot_function(){
	system_reboot(REBOOT_USERAPP);
} */
double refVx, refVy, refVz;

void loop() {
	//static int dataCount = 0;

	
	int syusoku;
	static int wait_count = 0;
	static byte pre_buttonstate = 0;
	
	byte swState = 0b00000000;

	char state = 0b00000000;

	/* if( !digitalRead
	(PIN_SW) ){
		reboot_function();
	} */
	

	if( flag_10ms ){
		int pathNum = motion.getPathNum();
		int conv;

		//------------------------------------上半身との通信------------------------------------
		unsigned short int len = Serial1.available(); // 何byte送られてきたかわかる？
		static char buff[10] = {};  // 送られてきた数字が一文字ずつ格納
		static int co = 0;  // buffの配列の要素数をcountしてるだけ

		for (int i = len;i;i--){  // 送られてきた数分繰り返す
			
			buff[co] = Serial1.read();
			if(buff[co] == '\n' /*&& buff[co-2] == 'U'*/) {
			state = buff[co-1]; // 改行コマンド送る前にstateを送ってる
			for (int j = co;j;j--){
				buff[j] = 0;  // 全部終わったら0に書き換える
			}
			co=0;
			}else;
			
			co++;
			
			if (co > 9){
			for (int j = 0;j <= 9;j++){
				buff[j] = 0;
			}
			co=0;
			}else;
		}
		//------------------------------------上半身との通信------------------------------------
		
		///// phase 0 /////////////////////////////////////////////////////////////////////////
		if(phase == 0){
			refVx = 0.0;
			refVy = 0.0;
			refVz = 0.0;
			if(pre_buttonstate == 0x0F && digitalRead(PIN_BUTTON1) == 1){ // スイッチの立ち上がりを検出してフェーズ移行
				if(retry_num == 10){
					gPosiz = 2.35619449;
					gPosix = -0.5;
					gPosiy =  0.5;
				}else if(retry_num == 0){
					gPosiz = 0.785398;//1.5708;//0;
					gPosix = 0.5;
					gPosiy = 0.5;
				}
				phase = 1;
			}
		///// phase 1 /////////////////////////////////////////////////////////////////////////
		}else if(phase == 1){ // ゲルゲ受け渡しまで
			// ----------------------------------上半身との通信----------------------------------
			if( pathNum == 8 && cmd != BIT_DEP ){
				cmd = BIT_DEP;
				Serial1.print('L');
				Serial1.print(cmd); // ゲルゲ展開
				Serial1.print('\n');
			}else if( pathNum == 10 && cmd != BIT_STOR ){
				cmd = BIT_STOR;
				Serial1.print('L');
				Serial1.print(cmd); // ゲルゲ展開
				Serial1.print('\n');
			}
			// ----------------------------------上半身との通信----------------------------------

			if(motion.getMode() != FOLLOW_TANGENT) motion.setMode(FOLLOW_TANGENT); // 接線方向を向くモードになっていなかったら変更
			
			syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる
			
			if(syusoku == 1){
				if( pathNum <= STATE1_1 ){
					motion.Px[3*pathNum+3] = gPosix;
					motion.Py[3*pathNum+3] = gPosiy;
					motion.incrPathnum(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値

					if( pathNum == STATE1_1 ) phase = 20;
				}
			}else if(syusoku == 0){ // 0の時は問題がないとき
				refVx = motion.refVx;
				refVy = motion.refVy;
				refVz = motion.refVz;
			}else{ // それ以外は問題ありなので止める
				refVx = 0.0;
				refVy = 0.0;
				refVz = 0.0;
			}
			
		///// phase 20 /////////////////////////////////////////////////////////////////////////
		}else if(phase == 20){
			// ----------------------------------上半身との通信----------------------------------
			if( pathNum == 8 && cmd != BIT_DEP ){
				cmd = BIT_DEP;
				Serial1.print('L');
				Serial1.print(cmd); // ゲルゲ展開
				Serial1.print('\n');
			}else if( pathNum == 10 && cmd != BIT_STOR ){
				cmd = BIT_STOR;
				Serial1.print('L');
				Serial1.print(cmd); // ゲルゲ展開
				Serial1.print('\n');
			}
			// ----------------------------------上半身との通信----------------------------------

			if(motion.getMode() != FOLLOW_COMMAND) motion.setMode(FOLLOW_COMMAND); // 接線方向を向くのではなく，指定した角度で
			
			syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる
			
			if(syusoku == 1){
				if( pathNum <= STATE1_2 ){
					motion.Px[3*pathNum+3] = gPosix;
					motion.Py[3*pathNum+3] = gPosiy;
					motion.incrPathnum(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値

					if( pathNum == STATE1_2 ) phase = 21;//phase = 2;
				}
			}else if(syusoku == 0){ // 0の時は問題がないとき
				refVx = motion.refVx;
				refVy = motion.refVy;
				refVz = motion.refVz;
			}else{ // それ以外は問題ありなので止める
				refVx = 0.0;
				refVy = 0.0;
				refVz = 0.0;
			}
		///// phase 21 /////////////////////////////////////////////////////////////////////////
		}else if(phase == 21){
			// ----------------------------------上半身との通信----------------------------------
			if( pathNum == 8 && cmd != BIT_DEP ){
				cmd = BIT_DEP;
				Serial1.print('L');
				Serial1.print(cmd); // ゲルゲ展開
				Serial1.print('\n');
			}else if( pathNum == 10 && cmd != BIT_STOR ){
				cmd = BIT_STOR;
				Serial1.print('L');
				Serial1.print(cmd); // ゲルゲ展開
				Serial1.print('\n');
			}
			// ----------------------------------上半身との通信----------------------------------

			if(motion.getMode() != FOLLOW_COMMAND) motion.setMode(FOLLOW_COMMAND); // 接線方向を向くのではなく，指定した角度で
			
			//syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる
			
			if( (!digitalRead(A0) && !digitalRead(A1)) || (!digitalRead(A4) && !digitalRead(A5)) ){
				if( pathNum <= STATE1_3 ){
					motion.Px[3*pathNum+3] = gPosix;
					motion.Py[3*pathNum+3] = gPosiy;
					motion.incrPathnum(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値

					if( pathNum == STATE1_3 ) phase = 22;//phase = 2;
				}
			}
		///// phase 22 /////////////////////////////////////////////////////////////////////////
		}else if(phase == 22){
			// ----------------------------------上半身との通信----------------------------------
			if( pathNum == 8 && cmd != BIT_DEP ){
				cmd = BIT_DEP;
				Serial1.print('L');
				Serial1.print(cmd); // ゲルゲ展開
				Serial1.print('\n');
			}
			// ----------------------------------上半身との通信----------------------------------

			if(zone == BLUE){
				refVy = 0.0;
				refVx = 0.15;
			}else{
				refVy = 0.0;
				refVx = 0.15;
			}

			if( fabs(gPosix) > 5.5 ){
				if( pathNum <= STATE1_3 ){
					motion.Px[3*pathNum+3] = gPosix;
					motion.Py[3*pathNum+3] = gPosiy;
					motion.incrPathnum(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値

					if( pathNum == STATE1_3 ) phase = 2;//phase = 2;
				}
			}
			
		///// phase 2 /////////////////////////////////////////////////////////////////////////
		}else if(phase == 2){ // じわじわ動いて位置補正
			if( cmd != BIT_STOR ){
				cmd = BIT_STOR;
				Serial1.print('L');
				Serial1.print(cmd); // ゲルゲ展開
				Serial1.print('\n');
			}
			// refVx = 0.0;
			// refVy = 0.0;
			// refVz = 0.0;

			// if(pre_buttonstate == 0 && digitalRead(PIN_BUTTON1) == 1){ // スイッチの立ち上がりを検出してフェーズ移行
			// 	//gPosix = 6.075;
			// 	//gPosiy = 8.225;
			// 	//motion.Px[3*pathNum+3] = gPosix;
			// 	//motion.Py[3*pathNum+3] = gPosiy;
			// 	motion.incrPathnum(0.02, 0.997);
			// 	phase = 3;
			// }
			//byte swState = 0b00000000;
			if( !digitalRead(A0) ) swState |= 0b00000001;	// 青用
			if( !digitalRead(A1) ) swState |= 0b00000010;	// 青用
			if( !digitalRead(A2) ) swState |= 0b00000100;	// 前
			if( !digitalRead(A3) ) swState |= 0b00001000;	// 前
			if( !digitalRead(A4) ) swState |= 0b00010000;	// 赤用
			if( !digitalRead(A5) ) swState |= 0b00100000;	// 赤用
			
			refVz = 0.0;

			if(zone == BLUE){
				// サイドのスイッチが押されていた場合
				if((swState & 0b00000011) == 0b00000011) refVy = 0.0;
				else refVy = -0.15;
				
				// フロントのスイッチが押されていた場合
				if((swState & 0b00001100) == 0b00001100) refVx = 0.0;
				else refVx = 0.15;
				
				// 両方押されていた場合
				if((swState & 0b00001111) == 0b00001111){
					wait_count++;
					if(wait_count >= 50){ //0.5秒間待つ
						gPosix = 6.058;
						gPosiy = 8.245;
						gPosiz = 0.0;

						// 次の位置PIDにおける目標値
						motion.Px[3 * pathNum] = 6.475;
						motion.Py[3 * pathNum] = 8.725;

						motion.incrPathnum(0.02, 0.997);
						phase = 3;
						wait_count = 0;
					}
					
				} else {
					wait_count = 0;
				}
			}else{
				// サイドのスイッチが押されていた場合
				if((swState & 0b00110000) == 0b00110000) refVy = 0.0;
				else refVy = 0.15;
				// フロントのスイッチが押されていた場合
				if((swState & 0b00001100) == 0b00001100) refVx = 0.0;
				else refVx = 0.15;
				
				// 両方押されていた場合
				if((swState & 0b00111100) == 0b00111100){
					wait_count++;
					if(wait_count >= 50){
						gPosix = -6.058; // ここは変えてね！
						gPosiy = 8.245;
						gPosiz = 3.1415;

						// 次の位置PIDにおける目標値　　ここも変更してね
						motion.Px[3 * pathNum] = -6.475;
						motion.Py[3 * pathNum] = 8.725;

						motion.incrPathnum(0.02, 0.997);
						phase = 3;
						wait_count = 0;
					}
				} else {
					wait_count = 0;
				}
			}
		}
		///// phase 3 /////////////////////////////////////////////////////////////////////////
		else if(phase == 3){ // 位置制御で壁から離れる
			if(motion.getMode() != POSITION_PID) motion.setMode(POSITION_PID);

			// リトライの時は速度を上げる
			if(retry_num == 1 || retry_num == 11){
				motion.refvel[pathNum] = 1.0;//1.2;
				motion.acc_count[pathNum] = 80;
			}

			syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる
			if(syusoku == 1){
				motion.Px[3*pathNum+3] = gPosix;
				motion.Py[3*pathNum+3] = gPosiy;
				//motion.refangle[pathNum+1] = 1.5708; // 次のフェーズでの目標角度
				motion.incrPathnum(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値

				phase = 4;
			}else if(syusoku == 0){ // 0の時は問題がないとき
				refVx = motion.refVx;
				refVy = motion.refVy;
				refVz = motion.refVz;
			}else{ // それ以外は問題ありなので止める
				refVx = 0.0;
				refVy = 0.0;
				refVz = 0.0;
			}
		///// phase 4 /////////////////////////////////////////////////////////////////////////
		}else if(phase == 4){ // 位置制御で角度を変える
			if(motion.getMode() != POSITION_PID) motion.setMode(POSITION_PID);

			syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる
			if(syusoku == 1){
				//digitalWrite(PIN_LED2, HIGH);
				motion.Px[3*pathNum+3] = gPosix;
				motion.Py[3*pathNum+3] = gPosiy;
				motion.incrPathnum(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値

				phase = 5;
			}else if(syusoku == 0){ // 0の時は問題がないとき
				refVx = motion.refVx;
				refVy = motion.refVy;
				refVz = motion.refVz;
			}else{ // それ以外は問題ありなので止める
				refVx = 0.0;
				refVy = 0.0;
				refVz = 0.0;
			}
		///// phase 5 /////////////////////////////////////////////////////////////////////////
		}else if(phase == 5){ // シャガイの前まで移動
			if( pathNum == 14 && cmd != BIT_DOWN ){
				cmd = BIT_DOWN;
				Serial1.print('L');
				Serial1.print(cmd); // シャガイを取るモード
				Serial1.print('\n');
			}

			if(motion.getMode() != FOLLOW_COMMAND) motion.setMode(FOLLOW_COMMAND); // 指定した方向を向くモードになっていなかったら変更

			if(retry_num == 1 || retry_num == 11){
				motion.refvel[pathNum] = 0.9;
				motion.acc_count[pathNum] = 15;
				motion.dec_tbe[pathNum] = 0.7;
			}
			
			syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる
			if(syusoku == 1){
				if( pathNum <= STATE2 ){
					motion.Px[3*pathNum+3] = gPosix;
					motion.Py[3*pathNum+3] = gPosiy;
					motion.incrPathnum(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値

					if( pathNum == STATE2 ) phase = 6;
				}
			}else if(syusoku == 0){ // 0の時は問題がないとき
				refVx = motion.refVx;
				refVy = motion.refVy;
				refVz = motion.refVz;
			}else{ // それ以外は問題ありなので止める
				refVx = 0.0;
				refVy = 0.0;
				refVz = 0.0;
			}
		///// phase 6 /////////////////////////////////////////////////////////////////////////
		}else if(phase == 6){ // シャガイの前で待機(本当はシャガイハンド上げたら次のフェーズに移行)
			cmd = BIT_0;
			Serial1.print('L');
			Serial1.print(cmd);
			Serial1.print('\n');

			refVx = 0.0;
			refVy = 0.0;
			refVz = 0.0;

			if( ( MASK_SHAGAIARM & state ) == MASK_SHAGAIARM ){ // シャガイハンドが下がっている
				//digitalWrite(PIN_LED2, LOW);
				if(digitalRead(PIN_BUTTON1) == 0) refVx = 0.1;
			}else if( ( MASK_SHAGAIARM & state ) == 0b00000000 ){ // シャガイハンドが上がっている
				//digitalWrite(PIN_LED2, HIGH);
				if(pre_buttonstate == 0x0F && digitalRead(PIN_BUTTON1)){ // スイッチの立ち上がりを検出してフェーズ移行
					phase = 7;
				}
			}	
		///// phase 7 /////////////////////////////////////////////////////////////////////////	
		}else if(phase == 7){ // スローイングゾーン前まで移動
			if(motion.getMode() != FOLLOW_COMMAND) motion.setMode(FOLLOW_COMMAND); // 指定した方向を向くモードになっていなかったら変更
			
			if(retry_num == 1 || retry_num == 11){
				motion.refvel[pathNum] = 0.9;
				motion.acc_count[pathNum] = 15;
				motion.dec_tbe[pathNum] = 0.7;
			}

			syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる
			if(syusoku == 1){
				if( pathNum <= STATE3 ){
					motion.Px[3*pathNum+3] = gPosix;
					motion.Py[3*pathNum+3] = gPosiy;
					motion.incrPathnum(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値

					if( pathNum == STATE3 ) phase = 8;
				}
			}else if(syusoku == 0){ // 0の時は問題がないとき
				refVx = motion.refVx;
				refVy = motion.refVy;
				refVz = motion.refVz;
			}else{ // それ以外は問題ありなので止める
				refVx = 0.0;
				refVy = 0.0;
				refVz = 0.0;
			}
		///// phase 8 /////////////////////////////////////////////////////////////////////////
		}else if(phase == 8){ // スイッチが押されるまでスローイングゾーン前で待機

			refVx = 0.0;
			refVy = 0.0;
			refVz = 0.0;

			if(pre_buttonstate == 0x0F && digitalRead(PIN_BUTTON1) == 1){ // スイッチの立ち上がりを検出してフェーズ移行
				phase = 9;
			}			
		///// phase 9 /////////////////////////////////////////////////////////////////////////	
		}else if(phase == 9){ // 投擲位置まで移動
			if( cmd != BIT_ROT ){
				cmd = BIT_ROT;
				Serial1.print('L');
				Serial1.print(cmd); // 投げる
				Serial1.print('\n');
			}
			
			if(motion.getMode() != FOLLOW_COMMAND) motion.setMode(FOLLOW_COMMAND); // 指定した方向を向くモードになっていなかったら変更
			
			syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる
			if(syusoku == 1){
				if( pathNum <= STATE4 ){
					motion.Px[3*pathNum+3] = gPosix;
					motion.Py[3*pathNum+3] = gPosiy;
					motion.incrPathnum(0.05, 0.997);//(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値

					if( pathNum == STATE4 ) phase = 10;
				}
			}else if(syusoku == 0){ // 0の時は問題がないとき
				refVx = motion.refVx;
				refVy = motion.refVy;
				refVz = motion.refVz;
			}else{ // それ以外は問題ありなので止める
				refVx = 0.0;
				refVy = 0.0;
				refVz = 0.0;
			}
		///// phase 10 /////////////////////////////////////////////////////////////////////////
		}else if(phase == 10){ // 投擲位置で待機(ここで投げる動作をする)
			// refVx = 0.0;
			// refVy = 0.0;
			// refVz = 0.0;
			if(motion.getMode() != POSITION_PID) motion.setMode(POSITION_PID);

			syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる

			if(syusoku == 1){
				//digitalWrite(PIN_LED2, HIGH);
				//motion.Px[3*pathNum+3] = gPosix;
				//motion.Py[3*pathNum+3] = gPosiy;

				// 収束した位置にとどめるために速度をゼロにする
				refVx = 0.0;
				refVy = 0.0;
				refVz = 0.0;

				// wait_count++;
				//if(wait_count == 5 && cmd != BIT_EXT){
					// cmd = BIT_EXT;
					// Serial1.print('L');
					// Serial1.print(cmd); // 投げる
					// Serial1.print('\n');
				//}

				// ボタンが押されていたら，次のフェーズへ　※場合によってはif文の外に出して，強制的に次のフェーズにした方がいいかも？
				if(pre_buttonstate == 0x0F && digitalRead(PIN_BUTTON1) == 1){ // スイッチの立ち上がりを検出してフェーズ移行
					digitalWrite(PIN_LED2, HIGH);
					// ボタンを押したら投擲を終了して，次のフェーズへ
					if( swTiming == 0 ){
						digitalWrite(PIN_LED1, HIGH);
						if(cmd != BIT_EXT){
							cmd = BIT_EXT;
							Serial1.print('L');
							Serial1.print(cmd); // 投げる
							Serial1.print('\n');
							swTiming++;
						}
					}else if( swTiming == 1 ){//if( !stopRoller ){		// スイッチを最初に押したとき
						stopRoller = true;
						if(cmd != BIT_DOWN){
							cmd = BIT_DOWN;
							Serial1.print('L');
							Serial1.print(cmd); // ハンド下げる
							Serial1.print('\n');
							wait_count = 0;
							swTiming++;
						}
					}else{					// スイッチをもう一度押したとき
						stopRoller = false;
						phase = 11;
						motion.incrPathnum(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値
						swTiming = 0;
					}
				}
				
			}else if(syusoku == 0){ // 0の時は問題がないとき
				refVx = motion.refVx;
				refVy = motion.refVy;
				refVz = motion.refVz;
			}else{ // それ以外は問題ありなので止める
				refVx = 0.0;
				refVy = 0.0;
				refVz = 0.0;
			}			
		///// phase 11 /////////////////////////////////////////////////////////////////////////	
		}else if(phase == 11){ // 2個目のシャガイの位置まで移動
			if( cmd != BIT_DOWN ){
				cmd = BIT_DOWN;
				Serial1.print('L');
				Serial1.print(cmd);
				Serial1.print('\n');
			}

			if(motion.getMode() != FOLLOW_COMMAND) motion.setMode(FOLLOW_COMMAND); // 指定した方向を向くモードになっていなかったら変更
			
			syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる
			if(syusoku == 1){
				if( pathNum <= STATE5 ){
					motion.Px[3*pathNum+3] = gPosix;
					motion.Py[3*pathNum+3] = gPosiy;
					motion.incrPathnum(0.05, 0.997);//(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値

					if( pathNum == STATE5 ) phase = 12;
				}
			}else if(syusoku == 0){ // 0の時は問題がないとき
				refVx = motion.refVx;
				refVy = motion.refVy;
				refVz = motion.refVz;
			}else{ // それ以外は問題ありなので止める
				refVx = 0.0;
				refVy = 0.0;
				refVz = 0.0;
			}
		///// phase 12 /////////////////////////////////////////////////////////////////////////
		}else if(phase == 12){ // 2個目のシャガイの前で待機(ここでシャガイを取得する一連の動作を行う)
			cmd = BIT_0;
			Serial1.print('L');
			Serial1.print(cmd);
			Serial1.print('\n');

			refVx = 0.0;
			refVy = 0.0;
			refVz = 0.0;

			if( ( MASK_SHAGAIARM & state ) == MASK_SHAGAIARM ){ // シャガイハンドが下がっている
				//digitalWrite(PIN_LED2, LOW);
				if(digitalRead(PIN_BUTTON1) == 0) refVx = 0.1;
			}else if( ( MASK_SHAGAIARM & state ) == 0b00000000 ){ // シャガイハンドが上がっている
				//digitalWrite(PIN_LED2, HIGH);
				if(pre_buttonstate == 0x0F && digitalRead(PIN_BUTTON1)){ // スイッチの立ち上がりを検出してフェーズ移行
					phase = 13;
				}
			}
			// if(pre_buttonstate == 0 && digitalRead(PIN_BUTTON1) == 1){ // スイッチの立ち上がりを検出してフェーズ移行
			// 	phase = 13;
			// }			
		///// phase 13 /////////////////////////////////////////////////////////////////////////	
		}else if(phase == 13){ // 2個目の投擲位置まで移動
			if( cmd != BIT_ROT ){
				cmd = BIT_ROT;
				Serial1.print('L');
				Serial1.print(cmd); // ローラ回転
				Serial1.print('\n');
			}
			
			if(motion.getMode() != FOLLOW_COMMAND) motion.setMode(FOLLOW_COMMAND); // 指定した方向を向くモードになっていなかったら変更
			
			syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる
			if(syusoku == 1){
				if( pathNum <= STATE6 ){
					motion.Px[3*pathNum+3] = gPosix;
					motion.Py[3*pathNum+3] = gPosiy;
					motion.incrPathnum(0.05, 0.997);//(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値

					if( pathNum == STATE6 ) phase = 14;
				}
			}else if(syusoku == 0){ // 0の時は問題がないとき
				refVx = motion.refVx;
				refVy = motion.refVy;
				refVz = motion.refVz;
			}else{ // それ以外は問題ありなので止める
				refVx = 0.0;
				refVy = 0.0;
				refVz = 0.0;
			}
		///// phase 14 /////////////////////////////////////////////////////////////////////////
		}else if(phase == 14){ // 投擲位置で待機(ここで投げる動作をする)
			if(motion.getMode() != POSITION_PID) motion.setMode(POSITION_PID);

			syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる

			if(syusoku == 1){
				//digitalWrite(PIN_LED2, HIGH);
				//motion.Px[3*pathNum+3] = gPosix;
				//motion.Py[3*pathNum+3] = gPosiy;

				// 収束した位置にとどめるために速度をゼロにする
				refVx = 0.0;
				refVy = 0.0;
				refVz = 0.0;

				// wait_count++;
				// if(wait_count >= 5 && cmd != BIT_EXT){
				// 	cmd = BIT_EXT;
				// 	Serial1.print('L');
				// 	Serial1.print(cmd); // 投げる
				// 	Serial1.print('\n');
				// }

				// ボタンが押されていたら，次のフェーズへ　※場合によってはif文の外に出して，強制的に次のフェーズにした方がいいかも？
				if(pre_buttonstate == 0x0F && digitalRead(PIN_BUTTON1) == 1){ // スイッチの立ち上がりを検出してフェーズ移行
					// ボタンを押したら投擲を終了して，次のフェーズへ
					if( swTiming == 0 ){
						cmd = BIT_EXT;
						Serial1.print('L');
						Serial1.print(cmd); // 投げる
						Serial1.print('\n');
						swTiming++;
					}else if( swTiming == 1 ){//if( !stopRoller ){		// スイッチを最初に押したとき
						stopRoller = true;

						cmd = BIT_DOWN;
						Serial1.print('L');
						Serial1.print(cmd); // ハンド下げる
						Serial1.print('\n');
						wait_count = 0;
						swTiming++;
					}else{					// スイッチをもう一度押したとき
						stopRoller = false;
						phase = 15;
						motion.incrPathnum(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値
						swTiming = 0;
					}
				}
				
			}else if(syusoku == 0){ // 0の時は問題がないとき
				refVx = motion.refVx;
				refVy = motion.refVy;
				refVz = motion.refVz;
			}else{ // それ以外は問題ありなので止める
				refVx = 0.0;
				refVy = 0.0;
				refVz = 0.0;
			}
		///// phase 15 /////////////////////////////////////////////////////////////////////////	
		}else if(phase == 15){ // 3個目のシャガイの位置まで移動
			if( cmd != BIT_DOWN ){
				cmd = BIT_DOWN;
				Serial1.print('L');
				Serial1.print(cmd);
				Serial1.print('\n');
			}
			
			if(motion.getMode() != FOLLOW_COMMAND) motion.setMode(FOLLOW_COMMAND); // 指定した方向を向くモードになっていなかったら変更
			
			syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる
			if(syusoku == 1){
				if( pathNum <= STATE7 ){
					motion.Px[3*pathNum+3] = gPosix;
					motion.Py[3*pathNum+3] = gPosiy;
					motion.incrPathnum(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値

					if( pathNum == STATE7 ) phase = 16;
				}
			}else if(syusoku == 0){ // 0の時は問題がないとき
				refVx = motion.refVx;
				refVy = motion.refVy;
				refVz = motion.refVz;
			}else{ // それ以外は問題ありなので止める
				refVx = 0.0;
				refVy = 0.0;
				refVz = 0.0;
			}
		///// phase 16 /////////////////////////////////////////////////////////////////////////
		}else if(phase == 16){ // 3個目のシャガイの前で待機(ここでシャガイを取得する一連の動作を行う)
			cmd = BIT_0;
			Serial1.print('L');
			Serial1.print(cmd);
			Serial1.print('\n');

			refVx = 0.0;
			refVy = 0.0;
			refVz = 0.0;

			if( ( MASK_SHAGAIARM & state ) == MASK_SHAGAIARM ){ // シャガイハンドが下がっている
				//digitalWrite(PIN_LED2, LOW);
				if(digitalRead(PIN_BUTTON1) == 0) refVx = 0.1;
			}else if( ( MASK_SHAGAIARM & state ) == 0b00000000 ){ // シャガイハンドが上がっている
				//digitalWrite(PIN_LED2, HIGH);
				if(pre_buttonstate == 0x0F && digitalRead(PIN_BUTTON1)){ // スイッチの立ち上がりを検出してフェーズ移行
					phase = 17;
				}
			}
		///// phase 17 /////////////////////////////////////////////////////////////////////////	
		}else if(phase == 17){ // 投擲位置まで移動
			if( cmd != BIT_ROT ){
				cmd = BIT_ROT;
				Serial1.print('L');
				Serial1.print(cmd); // 投げる
				Serial1.print('\n');
			}
			
			if(motion.getMode() != FOLLOW_COMMAND) motion.setMode(FOLLOW_COMMAND); // 指定した方向を向くモードになっていなかったら変更
			
			syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる
			if(syusoku == 1){
				if( pathNum <= STATE8 ){
					motion.Px[3*pathNum+3] = gPosix;
					motion.Py[3*pathNum+3] = gPosiy;
					motion.incrPathnum(0.05, 0.997);//(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値

					if( pathNum == STATE8 ) phase = 18;
				}
			}else if(syusoku == 0){ // 0の時は問題がないとき
				refVx = motion.refVx;
				refVy = motion.refVy;
				refVz = motion.refVz;
			}else{ // それ以外は問題ありなので止める
				refVx = 0.0;
				refVy = 0.0;
				refVz = 0.0;
			}
		///// phase 18 /////////////////////////////////////////////////////////////////////////
		}else if(phase == 18){ // 投擲位置で待機(ここで投げる動作をする)
			if(motion.getMode() != POSITION_PID) motion.setMode(POSITION_PID);

			syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる

			if(syusoku == 1){
				//digitalWrite(PIN_LED2, HIGH);
				//motion.Px[3*pathNum+3] = gPosix;
				//motion.Py[3*pathNum+3] = gPosiy;

				// 収束した位置にとどめるために速度をゼロにする
				refVx = 0.0;
				refVy = 0.0;
				refVz = 0.0;

				// wait_count++;
				// if(wait_count >= 5 && cmd != BIT_EXT){
				// 	cmd = BIT_EXT;
				// 	Serial1.print('L');
				// 	Serial1.print(cmd); // 投げる
				// 	Serial1.print('\n');
				// }

				// ボタンが押されていたら，次のフェーズへ　※場合によってはif文の外に出して，強制的に次のフェーズにした方がいいかも？
							
			}else if(syusoku == 0){ // 0の時は問題がないとき
				refVx = motion.refVx;
				refVy = motion.refVy;
				refVz = motion.refVz;
			}else{ // それ以外は問題ありなので止める
				refVx = 0.0;
				refVy = 0.0;
				refVz = 0.0;
			}

			if(pre_buttonstate == 0x0F && digitalRead(PIN_BUTTON1) == 1){ // スイッチの立ち上がりを検出してフェーズ移行
				// ボタンを押したら投擲を終了して，次のフェーズへ
				if( swTiming == 0 ){
					cmd = BIT_EXT;
					Serial1.print('L');
					Serial1.print(cmd); // 投げる
					Serial1.print('\n');
					swTiming++;
				}else if( swTiming == 1 ){//if( !stopRoller ){		// スイッチを最初に押したとき
					stopRoller = true;

					cmd = BIT_DOWN;
					Serial1.print('L');
					Serial1.print(cmd); // ハンド下げる
					Serial1.print('\n');
					wait_count = 0;
					swTiming++;
				}else{					// スイッチをもう一度押したとき
					stopRoller = false;
					//phase = 15;
					motion.incrPathnum(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値
					swTiming = 0;
				}
			}
		
		///// phase 100  リトライ用　 /////////////////////////////////////////////////////////////////////////			
		}else if(phase == 100){ // リスタート位置(線をまたいだ位置)で待機
			refVx = 0.0;
			refVy = 0.0;
			refVz = 0.0;

			if( cmd != BIT_DEP ){
				cmd = BIT_DEP;
				Serial1.print('L');
				Serial1.print(cmd); // ゲルゲ展開
				Serial1.print('\n');
			}

			if(pre_buttonstate == 0x0F && digitalRead(PIN_BUTTON1) == 1){ // スイッチの立ち上がりを検出してフェーズ移行
				if(retry_num == 11){
					gPosiz = 3.14159265;
					gPosix = -0.5;
					gPosiy =  0.5;
				}else if(retry_num == 1){
					gPosiz = 0.0;
					gPosix = 0.5;
					gPosiy = 0.5;
				}

				phase = 101;
				//digitalWrite(PIN_EXLED1, HIGH);
			}
		///// phase 101  リトライ用　 /////////////////////////////////////////////////////////////////////////
		}else if(phase == 101){ // 投擲位置まで移動
			refVx = 0.8; // とりあえずまっすぐ進むだけ
			refVy = 0.0;
			refVz = 0.0;
			if(zone == BLUE){
				if(gPosix >= 1.7){
					cmd = BIT_STOR;
					Serial1.print('L');
					Serial1.print(cmd); // ゲルゲ展開
					Serial1.print('\n');
					motion.setPathNum(11);
					phase = 2;
				}
			}else if(zone == RED){
				if(gPosix <= -1.7){
					cmd = BIT_STOR;
					Serial1.print('L');
					Serial1.print(cmd); // ゲルゲ展開
					Serial1.print('\n');
					motion.setPathNum(11);
					phase = 2;
				}
			}
			
		}

		// // ベジエ曲線
		// if( mode ){//if( phase < 10 ){
		// 	static int kari = 0;
		// if( kari < 51 ){
		// 	Serial.print(kari);
		// 	Serial.print("\t");
		// 	Serial.print(Px[kari]);
		// 	Serial.print("\t");
		// 	Serial.println(Py[kari]);
		// 	kari++;
		// }
		// 	pidPreError_update = false; // 位置制御モードになったら最初だけpreErrorを現在の値で作成
			
		// 	if( phase < ( STATE1 - 1) ){ // スラロームからゲルゲ受け渡しまで
		// 		// 接線方向を向くモードになっていなかったら変更
		// 		if(motion.getMode() != FOLLOW_TANGENT) motion.setMode(FOLLOW_TANGENT);
        // 	}else{
		// 		// 指令した方向を向くモードになっていなかったら変更
		// 		if(motion.getMode() != FOLLOW_COMMAND) motion.setMode(FOLLOW_COMMAND);				
        // 	}
			
		// 	int conv = motion.calc_refvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる
		// 	if(conv == 1){
		// 		int pathNum = motion.getPathNum();
		// 		if( pathNum < STATE1 ){
		// 			Px[3*pathNum+3] = gPosix;
		// 			Py[3*pathNum+3] = gPosiy;
		// 			motion.incrPhase(0.02, 0.997);
		// 		}else if( pathNum == STATE1 ){
		// 			flag_jiwajiwa = true;
		// 		}else{// 位置制御前は早めに次のフェーズへ
		// 			if(syusoku <= 0.3 || t_be >= 0.997){//(syusoku <= 0.25 || t_be >= 0.997){
		// 				//digitalWrite(PIN_LED2, HIGH);
		// 				Px[3*phase+3] = gPosix;
		// 				Py[3*phase+3] = gPosiy;
		// 				phase++;
		// 				pre_t_be = 0.1;
						
						
		// 				mode = false; // 位置制御モードに変更
		// 			}
		// 		}
		// 	}
			
			
		// // 位置制御　または　停止モード
		// } else {
		// 	//digitalWrite(PIN_LED1, HIGH);
		// 	// PIDのpreError更新(最初のみ)
		// 	if( !pidPreError_update ){
		// 		posiPIDx.PIDinit(Px[3*phase], gPosix);	// ref, act
		// 		posiPIDy.PIDinit(Py[3*phase], gPosiy);
		// 		posiPIDz.PIDinit(refangle[phase], gPosiz);
		// 		pidPreError_update = true; // ベジエモードに入ったらfalseになる．
		// 	}

		// 	// PIDクラスを使って位置制御を行う(速度の指令地を得る)
		// 	refVxg = posiPIDx.getCmd(Px[3*phase], gPosix, refvel[phase]);//(Px[30], gPosix, refvel[phase]);
		// 	refVyg = posiPIDy.getCmd(Py[3*phase], gPosiy, refvel[phase]);//(Py[30], gPosiy, refvel[phase]);
		// 	refVzg = posiPIDz.getCmd(refangle[phase], gPosiz, refvel[phase]);//(0.0, gPosiz, refvel[phase]);

		// 	// 上記はグローバル座標系における速度のため，ローカルに変換
		// 	refVx =  refVxg * cos(gPosiz) + refVyg * sin(gPosiz);
		// 	refVy = -refVxg * sin(gPosiz) + refVyg * cos(gPosiz);
		// 	refVz =  refVzg;

		// 	syusoku = sqrt(pow(gPosix-Px[3*phase], 2.0) + pow(gPosiy-Py[3*phase], 2.0));
		// 	if(syusoku <= 0.05){
		// 		//Px[3*phase+3] = gPosix;
		// 		//Py[3*phase+3] = gPosiy;
		// 		//phase++;
		// 		refVx = 0.0;
		// 		refVy = 0.0;
		// 		refVz = 0.0;

		// 		/* if( !digitalRead(A0) ){ // コントローラや上半身からの指令
		// 			mode = true; // ベジエモードに変更
		// 		} */
		// 	}
		// }

		// ローカル速度から，各車輪の角速度を計算
		double refOmegaA, refOmegaB, refOmegaC, refOmegaD;
		refOmegaA = ( refVx - refVy -refVz * ( _MECAHD_ADD_MECAHL ) ) / MECANUM_HANKEI;// 左前
		refOmegaB = ( refVx + refVy -refVz * ( _MECAHD_ADD_MECAHL ) ) / MECANUM_HANKEI;// 左後
		refOmegaC = ( refVx - refVy +refVz * ( _MECAHD_ADD_MECAHL ) ) / MECANUM_HANKEI;// 右後
		refOmegaD = ( refVx + refVy +refVz * ( _MECAHD_ADD_MECAHL ) ) / MECANUM_HANKEI;// 右前
		//double refSokudo = 0.1;// [ m/s ]
		//refOmegaA = refSokudo / MECANUM_HANKEI;
		//refOmegaB = refSokudo / MECANUM_HANKEI;
		//refOmegaC = refSokudo / MECANUM_HANKEI;
		//refOmegaD = refSokudo / MECANUM_HANKEI;

		double mdCmdA, mdCmdB, mdCmdC, mdCmdD;
		mdCmdA = refOmegaA * _2MECAR_PI;
		mdCmdB = refOmegaB * _2MECAR_PI;
		mdCmdC = refOmegaC * _2MECAR_PI;
		mdCmdD = refOmegaD * _2MECAR_PI;
		
		// 速度制御のためのコマンドをPIDクラスから得る
		// 最大値を超えていた場合に制限をかける

		static int dataFlag = 0;
		static int dataend = 0;
		/* if( dataFlag  || dataend ){
			mdCmdA = 0;
			mdCmdB = 0;
			mdCmdC = 0;
			mdCmdD = 0;
		} */

		

		// モータにcmd?を送り，回す
		MD.SpeedM1(ADR_MD1, -(int)mdCmdD);// 右前
		MD.SpeedM2(ADR_MD1,  (int)mdCmdA);// 左前
		MD.SpeedM1(ADR_MD2, -(int)mdCmdC);// 右後
		MD.SpeedM2(ADR_MD2,  (int)mdCmdB);// 左後

		/* static int printcount = 0;
		printcount++;
		if(printcount == 10){
			Serial.print( onx, 2 );
			Serial.print( "\t" );
			Serial.print( ony, 2 );
			Serial.print( "\t" );
			Serial.print( gPosix, 2 );
			Serial.print( "\t" );
			Serial.println( gPosiy, 2 );
			printcount = 0;
		} */
		//Serial.print( "\t" );
		//Serial.println(phase);//( gPosiz, 2 );

		/*** SDカード利用のために追加　2019/05/05 ***/
		/* String dataString = "";
		static bool first_write = true;
		if(first_write){
			dataString += "phase,onx,ony,gPosix,gPosiy,gPosiz,angle,dist";
			mySD.write_logdata(dataString);
			first_write = false;
			dataString = "";
		}
		dataString += String(phase) + "," + String(onx, 4) + "," + String(ony, 4);
		dataString += "," + String(gPosix, 4) + "," + String(gPosiy, 4) + "," + String(gPosiz, 4);
		dataString += "," + String(angle, 4)  + "," + String(dist, 4) + "," + String(refKakudo, 4) + "," + String(syusoku, 4) + "," + String(t_be, 4);
		
		mySD.write_logdata(dataString); */
		/*** SDカード利用のために追加　2019/05/05 ***/

		/* if( dataCount < 1200){//11990 ){
			*(pdata1 + dataCount) = ( int )( onx * 1000 );
			*(pdata2 + dataCount) = ( int )( ony * 1000 );
			*(pdata3 + dataCount) = ( int )( gPosix * 1000 );
			*(pdata4 + dataCount) = ( int )( gPosiy * 1000 );
			*(pdata5 + dataCount) = ( int )( angle * 1000 );
			*(pdata6 + dataCount) = ( int )( gPosiz * 1000 );
			*(pdata7 + dataCount) = ( int )( dist * 1000 );
			dataCount++;
		}else{
			//dataFlag = 1;
		} */



		/* if( !digitalRead(PIN_SW) && !dataend ){//if( dataFlag && !dataend ){
			dataend = true;
			for( int forcount = 0; forcount < 1200 ; forcount++ ){
				Serial.print( *(pdata1 + forcount) );
				Serial.print("\t");
				Serial.print( *(pdata2 + forcount) );
				Serial.print("\t");
				Serial.print( *(pdata3 + forcount) );
				Serial.print("\t");
				Serial.println( *(pdata4 + forcount) );
				//Serial.print("\t");
				Serial.print( *(pdata5 + forcount) );
				Serial.print("\t");
				Serial.print( *(pdata6 + forcount) );
				Serial.print("\t");
				Serial.println( *(pdata7 + forcount) );
				//if( forcount == 1199 ){
				//	dataend = true;
				//}
			}
		} */

		//Serial.print(onx * 1000);//xl
		//Serial.print("\t");
		//Serial.print(ony * 1000);//y
		//Serial.print("\t");
		/* Serial.print(gPosix * 1000);
		Serial.print("\t");
		Serial.println(gPosiy * 1000); */
		/* Serial.print(EncountA);
		Serial.print("\t");
		Serial.print(EncountB);
		Serial.print("\t");
		Serial.print(EncountC);
		Serial.print("\t");
		Serial.print(Posixl, 3);
		Serial.print("\t");
		Serial.println(Posixr, 3); */

		/* Serial.print(gPosix, 4);
		Serial.print("\t");
		Serial.print(gPosiy, 4);
		Serial.print("\t");
		Serial.println(gPosiz, 4); */

		
		/* Serial.print();
		Serial.print();
		Serial.print();
		Serial.print(); */

		
		/* Serial.print(phase);
		Serial.print("\t");
		Serial.print(refVx);
		Serial.print("\t");
		Serial.print(refVy);
		Serial.print("\t");
		Serial.println(swState, BIN); */
		/* Serial.print(gPosix, 4);
		Serial.print("\t");
		Serial.print(gPosiy, 4);
		Serial.print("\t");
		Serial.println(motion.calcRefvel(gPosix, gPosiy, gPosiz)); */
		
		//Serial.println(pre_buttonstate);

		pre_buttonstate = pre_buttonstate<<1;
		pre_buttonstate &= 0x0F;
		pre_buttonstate |= !digitalRead(PIN_BUTTON1);
		flag_10ms = false;
	}

	if( flag_20ms ){
		/* static int count_sens = 0;
		sensVal[count_sens] = gSensor[count_sens].readRangeSingleMillimeters();
		Serial.print(sensVal[count_sens]);
		Serial.print("\t");
		count_sens++;
		if(count_sens == SENSOR_NUM){
			count_sens = 0;
			Serial.println();
		} */

		/* for(int n = 0; n < SENSOR_NUM; n++){
			sensVal[n] = gSensor[n].readRangeSingleMillimeters();
			Serial.print(sensVal[n]);
			Serial.print("\t");
		}
		Serial.println(); */
		
		/*** SDカード利用のために追加　2019/05/05 ***/
		String dataString = "";
		static bool first_write = true;
		if(first_write){
			dataString += "phase,path_num,onx,ony,gPosix,gPosiy,gPosiz,angle,dist,refKakudo,refVx,refVy,refVz";
			mySD.write_logdata(dataString);
			first_write = false;
			dataString = "";
		}
		dataString += String(phase) + "," + String(motion.getPathNum()) + "," + String(motion.onx, 4) + "," + String(motion.ony, 4);
		dataString += "," + String(gPosix, 4) + "," + String(gPosiy, 4) + "," + String(gPosiz, 4);
		dataString += "," + String(motion.angle, 4)  + "," + String(motion.dist, 4) + "," + String(motion.refKakudo, 4);
		dataString += "," + String(refVx, 4) + "," + String(refVy, 4) + "," + String(refVz, 4);
		
		mySD.write_logdata(dataString);
		/*** SDカード利用のために追加　2019/05/05 ***/

		flag_20ms = false;
	}

	
}