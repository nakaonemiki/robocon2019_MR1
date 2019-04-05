/************************************************/
// 
// 長さの単位はmmとする
// 
/************************************************/

#include <Arduino.h>
#include <RoboClaw.h>
#include "define.h"
#include "MsTimerTPU3.h"
#include "phaseCounter.h"
#include "PIDclass.h"
#include "Filter.h"

phaseCounter Enc1(1);
phaseCounter Enc2(2);
phaseCounter Enc3(3);

// MDインスタンス化させたほうがいい?

PID velPIDA(0.0, 0.0, 0.0, INT_TIME);
PID velPIDB(0.0, 0.0, 0.0, INT_TIME);
PID velPIDC(0.0, 0.0, 0.0, INT_TIME);

PID posiPIDx(0.0, 0.0, 0.0, INT_TIME);
PID posiPIDy(0.0, 0.0, 0.0, INT_TIME);
PID posiPIDz(0.0, 0.0, 0.0, INT_TIME);

PID yokozurePID(0.0, 0.0, 0.0, INT_TIME);
PID kakudoPID(0.0, 0.0, 0.0, INT_TIME);

Filter sokduo_filter(INT_TIME);
Filter kakudo_filter(INT_TIME);

double Px[22] = { 0.50, 0.50, 1.94, 1.94, 1.94, 0.51, 0.51 };
double Py[22] = { 0.50, 1.25, 1.25, 2.00, 2.75, 2.75, 3.50 };

double refvel[7] = {0.5,0.5,0.5,0.5,0.3,0.5,0.5};

double Ax[7];
double Bx[7];
double Cx[7];
double Dx[7];

double Ay[7];
double By[7];
double Cy[7];
double Dy[7];

double a_be[7];
double b_be[7];
double c_be[7];
double d_be[7];
double e_be[7];
double f_be[7];
double d_be_[7];
double e_be_[7];
double f_be_[7];

double t_be = 0.0;
double pre_t_be = 0.1;
double epsilon = 1.0;

double refVxg, refVyg, refVzg; // グローバル座標系の指定速度
int EncountA, EncountB, EncountC;
int pre_EncountA = 0, pre_EncountB = 0, preAngleC = 0;


// グローバル変数の設定
double gPosix = Px[0], gPosiy = Py[0], gPosiz = 0;


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
    static double preAngleA = 0.0, preAngleB = 0.0, preAngleC = 0.0;
    static double preVxl = 0.0, preVyl = 0.0, preVzl = 0.0;
    static int phase = 0;
    count++;

    // 自己位置推定用エンコーダのカウント値取得
    EncountA = Enc1.getCount();	// MTU1, xr
    EncountB = Enc2.getCount();	// MTU2, y
    EncountC = Enc3.getCount();	// TPU1, xl
	
	// 角度   encountはdoubleに型変換した方がいいかもしれない
	Kakudoxl = ( double )( EncountC - pre_EncountC ) * 2.0 * PI / MEASURE_RES_MUL_X;
	Kakudoxr = ( double )( EncountA - pre_EncountA ) * 2.0 * PI / MEASURE_RES_MUL_X;
	Kakudoy  = ( double )( EncountB - pre_EncountB ) * 2.0 * PI / MEASURE_RES_MUL_Y;
	
	// ローカル用(zは角度)
	Posix = MEASURE_HANKEI * 0.5 * ( Kakudoxl + Kakudoxr );
	Posiy = ( MEASURE_HANKEI / 3.0 ) * ( ( 2.0 * Kakudoy ) + ( ( HANKEI_L * ( Kakudoxr - Kakudoxl ) ) / HANKEI_D ) );
	Posiz = ( MEASURE_HANKEI / 3.0 ) * ( ( Kakudoy / HANKEI_L ) + ( ( Kakudoxr - Kakudoxl ) / HANKEI_D ) );
	
	// グローバル用(zは角度)
	double tmp_Posiz = gPosi + ( Posiz * 0.5 );	// つまりgPosi + ( Posiz / 2.0 );
	gPosix += Posix * cos( tmp_Posiz ) - Posiy * sin( tmp_Posiz ) );
	gPosiy += Posix * sin( tmp_Posiz ) + Posiy * cos( tmp_Posiz ) );
	gPosiz += Posiz;
	
	
	pre_EncountA = EncountA;
	pre_EncountB = EncountB;
	pre_EncountC = EncountC;
}


void setup() {
	pinMode(PIN_SW, INPUT);		// reboot用
	pinMode(PIN_LED0, OUTPUT);
    pinMode(PIN_LED1, OUTPUT);
	pinMode(PIN_LED2, OUTPUT);
    pinMode(PIN_LED3, OUTPUT);
	
	// 自己位置推定用のエンコーダ
	Enc1.init();
	Enc2.init();
	Enc3.init();
	
	// PCと通信
	Serial.begin(115200);
	
	//
	velPIDA.PIDinit(0.0, 0.0);
	velPIDB.PIDinit(0.0, 0.0);
	velPIDC.PIDinit(0.0, 0.0);
	
	posiPIDx.PIDinit(0.0, 0.0);
	posiPIDy.PIDinit(0.0, 0.0);
	posiPIDz.PIDinit(0.0, 0.0);
	
	yokozurePID.PIDinit(0.0, 0.0);
	kakudoPID.PIDinit(0.0, 0.0);
	
	// タイマー割り込み(とりあえず10ms)
	MsTimerTPU3::set((int)(INT_TIME * 1000), timer_warikomi); // 10ms period
	MsTimerTPU3::start();
}


void reboot_function(){
	system_reboot(REBOOT_USERAPP);
}


void loop() {
	if(!digitalRead(PIN_SW)){
		reboot_function();
	}
}
