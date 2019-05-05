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


// 自己位置推定用のエンコーダ
phaseCounter Enc1(1);
phaseCounter Enc2(2);
phaseCounter Enc3(3);

// RoboClaw
RoboClaw MD(&Serial2,1);//10);

// lpms-me1
lpms_me1 lpms(&Serial3);

PID posiPIDx(7.0, 0.0, 0.0, INT_TIME);
PID posiPIDy(6.0, 0.0, 0.0, INT_TIME);
PID posiPIDz(9.0, 0.0, 0.0, INT_TIME);

PID yokozurePID(3.0, 0.0, 0.0, INT_TIME);
PID kakudoPID(3.0, 0.0, 0.0, INT_TIME);

// 二次遅れ使えるようになる
Filter sokduo_filter(INT_TIME);
Filter kakudo_filter(INT_TIME);

mySDclass mySD;

// VL53L0X
/*const int VL53L0X_GPIO[SENSOR_NUM] = {A0, A1, A2, A3};
VL53L0X gSensor[SENSOR_NUM]; // 使用するセンサークラス配列
unsigned int sensVal[SENSOR_NUM] = {0};*/

// ベジエ曲線用
double Px[ STATE_ALL * 3 + 1 ] = //Px[31] = /* P0が頭 */
	/* 0 */{ 0.500, 1.000, 1.225,
	/* 1 */1.475, 1.725, 1.725, 
	/* 2 */1.475, 1.225, 1.165, 
	/* 3 */0.945, 0.725, 0.725, 
	/* 4 */0.945, 1.045, 1.225, 
	/* 5 */1.475, 1.725, 1.600, 
	/* 6 */1.400, 1.200, 1.050, 
	/* 7 */1.225, 1.400, 1.800,
	/* 8 */2.300, 2.800, 3.500,
	/* 9 */4.000, 5.000, 5.500,
	/* 10 */6.050 };
double Py[ STATE_ALL * 3 + 1 ] = //Py[31] = 
	/* 0 */{ 0.500, 1.038, 1.281, 
	/* 1 */1.550, 1.819, 2.167, 
	/* 2 */2.450, 2.733, 2.801, 
	/* 3 */3.050, 3.299, 3.722, 
	/* 4 */3.950, 4.054, 4.241, 
	/* 5 */4.500, 4.759, 5.400, 
	/* 6 */5.800, 6.200, 7.850, 
	/* 7 */8.200, 8.550, 8.574,
	/* 8 */8.500, 8.426, 8.250,
	/* 9 */8.250, 8.250, 8.250,
	/* 10 */8.250 };

const double _straight = 1.0;
const double _curve = 1.0;
const double _other = 1.0;

double refvel[ STATE_ALL ] = {/*A*/_straight,/*B*/_curve,/*C*/_straight,/*D*/_curve,/*E*/_straight,/*F*/_curve,/*G*/_other,/*H*/_other,/*I*/_other,/*J*/_other};
//{/*A*/1.5,/*B*/1.0,/*C*/1.5,/*D*/1.0,/*E*/1.5,/*F*/1.0,/*G*/1.1,/*H*/0.8,/*I*/0.6,/*J*/0.6};
//{/*A*/0.3,/*B*/0.3,/*C*/0.3,/*D*/0.3,/*E*/0.3,/*F*/0.3,/*G*/0.3,/*H*/0.3,/*I*/0.3,/*J*/0.3};//{/*A*/1.0,/*B*/1.0,/*C*/1.0,/*D*/1.0,/*E*/1.0,/*F*/1.0,/*G*/1.0,/*H*/1.0,/*I*/1.0,/*J*/1.0};
//{/*A*/1.2,/*B*/1.0,/*C*/1.2,/*D*/1.0,/*E*/1.2,/*F*/1.0,/*G*/1.1,/*H*/1.0,/*I*/1.1,/*J*/1.2};
//{/*A*/1.5,/*B*/0.7,/*C*/1.5,/*D*/0.7,/*E*/1.5,/*F*/0.7,/*G*/1.0,/*H*/0.7,/*I*/0.7,/*J*/0.7};

/*** SDカード利用のためにちゅいか　2019/05/05 ***/
double Px_SD[ STATE_ALL * 3 + 1 ], Py_SD[ STATE_ALL * 3 + 1 ], refvel_SD[ STATE_ALL ], refkakudo_SD[ STATE_ALL ];
/*** SDカード利用のためにちゅいか　2019/05/05 ***/

// ベジエ曲線関連
double Ax[ STATE_ALL ];
double Bx[ STATE_ALL ];
double Cx[ STATE_ALL ];
double Dx[ STATE_ALL ];

double Ay[ STATE_ALL ];
double By[ STATE_ALL ];
double Cy[ STATE_ALL ];
double Dy[ STATE_ALL ];

// 内積関連
double a_be[ STATE_ALL ];
double b_be[ STATE_ALL ];
double c_be[ STATE_ALL ];
double d_be[ STATE_ALL ];
double e_be[ STATE_ALL ];
double f_be[ STATE_ALL ];
double d_be_[ STATE_ALL ];
double e_be_[ STATE_ALL ];
double f_be_[ STATE_ALL ];

double t_be = 0.0;
double pre_t_be = 0.1;
double epsilon = 1.0;

double refVxg, refVyg, refVzg; // グローバル座標系の指定速度
int EncountA = 0, EncountB = 0, EncountC = 0;
int pre_EncountA = 0, pre_EncountB = 0, preAngleC = 0;
int pre_tmpEncA = 0, pre_tmpEncB = 0, pre_tmpEncC = 0;
int ledcount = 0;

int count_10ms = 0;//0;
boolean flag_10ms = false;
boolean flag_20ms = false;
boolean flag_5s = false;

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

/* double Kakudoxl, Kakudoxr, Kakudoy, tmpKakudoy;
double Posix, Posiy, Posiz; */
double tmpPosix = 0.0, tmpPosiy = 0.0, tmpPosiz = 0.0;


// グローバル変数の設定
double gPosix = Px[0], gPosiy = Py[0], gPosiz = 0.785398;//1.5708;//0;
double angle_rad = gPosiz;
const double _ANGLE_DEG = 45.0;

// tを求めるための方程式
double func(int p, double t)
{
    return a_be[p] * pow(t, 5.0) + b_be[p] * pow(t,4.0) + c_be[p] * pow(t,3.0) + d_be[p] * pow(t,2.0) + e_be[p] * t + f_be[p];
}
// tを求めるための方程式の1階微分
double dfunc(int p, double t)
{
    return 5.0 * a_be[p] * pow(t, 4.0) +  4.0 * b_be[p] * pow(t,3.0) + 3.0 * c_be[p] * pow(t,2.0) + 2.0 * d_be[p] * t + e_be[p];
}

// tにおけるベジエ曲線の座標を求める関数
double bezier_x(int p, double t)
{
    return Ax[p]*pow(t,3.0) + 3.0*Bx[p]*pow(t,2.0) + 3.0*Cx[p]*t + Dx[p];
}
double bezier_y(int p, double t)
{
    return Ay[p]*pow(t,3.0) + 3.0*By[p]*pow(t,2.0) + 3.0*Cy[p]*t + Dy[p];
}

// ベジエ曲線式の1階微分
double dbezier_x(int p, double t)
{
    return 3.0*Ax[p]*pow(t,2.0) + 6.0*Bx[p]*t + 3.0*Cx[p];
}
double dbezier_y(int p, double t)
{
    return 3.0*Ay[p]*pow(t,2.0) + 6.0*By[p]*t + 3.0*Cy[p];
}

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
		} else {
			digitalWrite(PIN_LED0, LOW);
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
	if(count_5s >= 500){
		flag_5s = true;
	}
	if( flag_5s ){
		count_10ms++;
		if( count_10ms == 1 ){
			flag_10ms = true;
			count_10ms = 0;
		}
	}
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

	// RoboClaw
	MD.begin(115200);

	// PCと通信
	Serial.begin(230400);
	
	if(lpms.init() == 0){
		digitalWrite(PIN_LED3, HIGH);
	}
	//Serial.println(lpms.init());
	
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

	// PID関連初期化
	posiPIDx.PIDinit(0.0, 0.0);
	posiPIDy.PIDinit(0.0, 0.0);
	posiPIDz.PIDinit(0.0, 0.0);
	
	yokozurePID.PIDinit(0.0, 0.0);
	kakudoPID.PIDinit(0.0, 0.0);

	sokduo_filter.setSecondOrderPara(15.0, 1.0, 0.0);
    //kakudo_filter.setSecondOrderPara(10.0, 1.0, 0.0);

	for(int i = 0; i < STATE_ALL; i++) {
        Ax[i] = Px[3*i+3] -3*Px[3*i+2] + 3*Px[3*i+1] - Px[3*i+0];
        Ay[i] = Py[3*i+3] -3*Py[3*i+2] + 3*Py[3*i+1] - Py[3*i+0];
        Bx[i] = Px[3*i+2] -2*Px[3*i+1] + Px[3*i+0];
        By[i] = Py[3*i+2] -2*Py[3*i+1] + Py[3*i+0];
        Cx[i] = Px[3*i+1] - Px[3*i+0];
        Cy[i] = Py[3*i+1] - Py[3*i+0];
        Dx[i] = Px[3*i+0];
        Dy[i] = Py[3*i+0];
    }

    for(int i = 0; i < STATE_ALL; i++) {
        a_be[i] = pow(Ax[i], 2.0) + pow(Ay[i], 2.0);
        b_be[i] = 5*(Ax[i]*Bx[i] + Ay[i]*By[i]);
        c_be[i] = 2*((3*pow(Bx[i],2.0)+2*Ax[i]*Cx[i]) + (3*pow(By[i],2.0)+2*Ay[i]*Cy[i]));
        d_be_[i] = 9*Bx[i]*Cx[i] + 9*By[i]*Cy[i];
        e_be_[i] = 3*pow(Cx[i],2.0) + 3*pow(Cy[i],2.0);
        f_be_[i] = 0;
    }

	/*** SDカード利用のためにちゅいか　2019/05/05 ***/
	Serial.print("Initializing ...");
	//Serial.println(mySD.init());
	mySD.init();
	Serial.print("Path reading ...");
	//Serial.println(mySD.path_read(BLUE, Px, Py, Vel, Angle));
	mySD.path_read(BLUE, Px_SD, Py_SD, refvel_SD, refkakudo_SD);
	Serial.print("Log file making ...");
	//Serial.println(mySD.make_logfile());
	mySD.make_logfile();
	/*** SDカード利用のためにちゅいか　2019/05/05 ***/

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


void loop() {
	static int dataCount = 0;

	/* if( !digitalRead(PIN_SW) ){
		reboot_function();
	} */
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
	}

	flag_20ms = false;

	if( flag_10ms ){		
		static int phase = 0;
		double refVx, refVy, refVz;

		double onx, ony;    //ベジエ曲線上の点

		double angle, dist;

		// ベジエ曲線
		if( phase < 10 ){
			double tmpx = Px[phase*3] - gPosix;
			double tmpy = Py[phase*3] - gPosiy;
			
			d_be[phase] = d_be_[phase] + Ax[phase] * tmpx + Ay[phase] * tmpy;
			e_be[phase] = e_be_[phase] + 2*Bx[phase] * tmpx + 2*By[phase] * tmpy;
			f_be[phase] = f_be_[phase] + Cx[phase] * tmpx + Cy[phase] * tmpy;
			
			int count_newton = 0;
			do {
				t_be = pre_t_be - func(phase, pre_t_be)/dfunc(phase, pre_t_be);
				epsilon = abs((t_be - pre_t_be)/pre_t_be);
				
				//if(t_be < 0) t_be = 0.0;
				//else if(t_be > 1.0) t_be = 1.0;
				
				pre_t_be = t_be;
				count_newton++;
			}while(epsilon >= 1e-4 && count_newton <= 50);
			
			//double onx, ony;    //ベジエ曲線上の点
			onx = bezier_x(phase, t_be);
			ony = bezier_y(phase, t_be);
			
			// 外積による距離導出
			//double angle;
			angle = atan2(dbezier_y(phase, t_be), dbezier_x(phase, t_be)); // ベジエ曲線の接線方向
			//double dist;
			dist = (ony - gPosiy)*cos(angle) - (onx - gPosix)*sin(angle);
			
			double refVtan, refVper, refVrot;
			refVtan = sokduo_filter.SecondOrderLag(refvel[phase]);
			refVper = yokozurePID.getCmd(dist, 0.0, refvel[phase]);
			//refKakudo = kakudo_filter.SecondOrderLag(refangle[phase]);
			refVrot = kakudoPID.getCmd(angle, gPosiz, 1.57);//(refKakudo, gPosiz, 1.57);
			
			// ローカル座標系の指令速度(グローバル座標系のも込み込み)
			refVx =  refVtan * cos( gPosiz - angle ) + refVper * sin( gPosiz - angle );
			refVy = -refVtan * sin( gPosiz - angle ) + refVper * cos( gPosiz - angle );
			refVz =  refVrot;//0.628319;だと10秒で旋回？
			
			double syusoku;
			syusoku = sqrt(pow(gPosix-Px[3*phase+3], 2.0) + pow(gPosiy-Py[3*phase+3], 2.0));
			if( phase < 9 ){
				if(syusoku <= 0.02 || t_be >= 0.997){//(syusoku <= 0.05 || t_be >= 0.997){
					digitalWrite(PIN_LED2, HIGH);
					Px[3*phase+3] = gPosix;
					Py[3*phase+3] = gPosiy;
					phase++;
					pre_t_be = 0.1;
				}
			}else{// 位置制御前は早めに次のフェーズへ
				if(syusoku <= 0.25 || t_be >= 0.997){
					Px[3*phase+3] = gPosix;
					Py[3*phase+3] = gPosiy;
					phase++;
					pre_t_be = 0.1;
				}
			}
			
			epsilon = 1.0;
		} else {
			// PIDクラスを使って位置制御を行う(速度の指令地を得る)
			refVxg = posiPIDx.getCmd(Px[30], gPosix, refvel[phase]);
			refVyg = posiPIDy.getCmd(Py[30], gPosiy, refvel[phase]);
			refVzg = posiPIDz.getCmd(0.0, gPosiz, refvel[phase]);

			// 上記はグローバル座標系における速度のため，ローカルに変換
			refVx =  refVxg * cos(gPosiz) + refVyg * sin(gPosiz);
			refVy = -refVxg * sin(gPosiz) + refVyg * cos(gPosiz);
			refVz =  refVzg;
		}

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
		MD.SpeedM1(ADR_MD1,  (int)mdCmdD);//-(int)mdCmdB);// (int)mdCmdB);// 左後
		MD.SpeedM2(ADR_MD1, -(int)mdCmdA);// (int)mdCmdC);//-(int)mdCmdC);// 右後
		MD.SpeedM1(ADR_MD2,  (int)mdCmdC);//-(int)mdCmdA);// (int)mdCmdA);// 左前
		MD.SpeedM2(ADR_MD2, -(int)mdCmdB);// (int)mdCmdD);//-(int)mdCmdD);// 右前

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

		/*** SDカード利用のためにちゅいか　2019/05/05 ***/
		String dataString = "";
		static bool first_write = true;
		if(first_write){
			dataString += "onx,ony,gPosix,gPosiy,gPosiz,angle,dist";
			mySD.write_logdata(dataString);
			first_write = false;
			dataString = "";
		}
		dataString += String(onx, 4) + "," + String(ony, 4);
		dataString += "," + String(gPosix, 4) + "," + String(gPosiy, 4) + "," + String(gPosiz, 4);
		dataString += "," + String(angle, 4)  + "," + String(dist, 4);
		
		mySD.write_logdata(dataString);
		/*** SDカード利用のためにちゅいか　2019/05/05 ***/

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

		/* Serial.print(Px[3], 4);
		Serial.print("\t");
		Serial.println(Py[3], 4); */
		//Serial.print("\t");
		//Serial.println(gPosiz);

		

		flag_10ms = false;
	}
}