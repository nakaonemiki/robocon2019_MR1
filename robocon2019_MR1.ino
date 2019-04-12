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
#include "reboot.h"

// 自己位置推定用のエンコーダ
phaseCounter Enc1(1);
phaseCounter Enc2(2);
phaseCounter Enc3(3);

// RoboClaw
RoboClaw MD(&Serial2,10);

PID velPIDA(0.0, 0.0, 0.0, INT_TIME);
PID velPIDB(0.0, 0.0, 0.0, INT_TIME);
PID velPIDC(0.0, 0.0, 0.0, INT_TIME);

PID posiPIDx(0.0, 0.0, 0.0, INT_TIME);
PID posiPIDy(0.0, 0.0, 0.0, INT_TIME);
PID posiPIDz(0.0, 0.0, 0.0, INT_TIME);

PID yokozurePID(0.0, 0.0, 0.0, INT_TIME);
PID kakudoPID(0.0, 0.0, 0.0, INT_TIME);

// 二次遅れ使えるようになる
Filter sokduo_filter(INT_TIME);
Filter kakudo_filter(INT_TIME);

// ベジエ曲線用
double Px[22] = { 0.50, 0.50, 1.94, 1.94, 1.94, 0.51, 0.51 };
double Py[22] = { 0.50, 1.25, 1.25, 2.00, 2.75, 2.75, 3.50 };

double refvel[7] = {0.5,0.5,0.5,0.5,0.3,0.5,0.5};

// ベジエ曲線関連
double Ax[7];
double Bx[7];
double Cx[7];
double Dx[7];

double Ay[7];
double By[7];
double Cy[7];
double Dy[7];

// 内積関連
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
int EncountA = 0, EncountB = 0, EncountC = 0;
int pre_EncountA = 0, pre_EncountB = 0, preAngleC = 0;
int ledcount = 0;


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

/* void Mecanum_command(double duty[4]){
  MD.SpeedM1(ADR_MD1, (int)(duty[3])*M_CMD);
  MD.SpeedM2(ADR_MD1, (int)(duty[2])*M_CMD);
  MD.SpeedM1(ADR_MD2, (int)(duty[1])*M_CMD);
  MD.SpeedM2(ADR_MD2, (int)(duty[0])*M_CMD);
} */

void timer_warikomi(){
	static int count = 0;
	static double pre_EncountA = 0.0, pre_EncountB = 0.0, pre_EncountC = 0.0;
    static double preAngleA = 0.0, preAngleB = 0.0, preAngleC = 0.0;
    static double preVxl = 0.0, preVyl = 0.0, preVzl = 0.0;
    static int phase = 0;
    count++;
	
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
    EncountA = -Enc1.getCount();	// MTU1, xl
    EncountB = -Enc2.getCount();	// MTU2, y
    EncountC = Enc3.getCount();	// TPU1, xr
	
	// 角度   encountはdoubleに型変換した方がいいかもしれない
	double Kakudoxl, Kakudoxr, Kakudoy;
	Kakudoxl = ( double )( EncountA - pre_EncountA ) * 2.0 * PI / MEASURE_RES_MUL_X;
	Kakudoxr = ( double )( EncountC - pre_EncountC ) * 2.0 * PI / MEASURE_RES_MUL_X;
	Kakudoy  = ( double )( EncountB - pre_EncountB ) * 2.0 * PI / MEASURE_RES_MUL_Y;
	
	// static double tmpKakudoxl = 0.0, tmpKakudoxr = 0.0, tmpKakudoy = 0.0;
	// tmpKakudoxl += Kakudoxl;
	// tmpKakudoxr += Kakudoxr;
	// tmpKakudoy  += Kakudoy;
	
	// ローカル用(zは角度)
	double Posix, Posiy, Posiz;
	Posiz = MEASURE_HANKEI * ( Kakudoxr - Kakudoxl ) * 0.5 / MEASURE_HANKEI_D;
	Posix = MEASURE_HANKEI * 0.5 * ( Kakudoxl + Kakudoxr );
	Posiy = MEASURE_HANKEI * Kakudoy - MEASURE_HANKEI_L * Posiz;//( MEASURE_HANKEI / 3.0 ) * ( ( 2.0 * Kakudoy ) + ( ( MEASURE_HANKEI_L * ( Kakudoxr - Kakudoxl ) ) / MEASURE_HANKEI_D ) );
	
	// グローバル用(zは角度)
	double tmp_Posiz = gPosiz + ( Posiz * 0.5 ); // つまりgPosi + ( Posiz / 2.0 );
	gPosix += Posix * cos( tmp_Posiz ) - Posiy * sin( tmp_Posiz );
	gPosiy += Posix * sin( tmp_Posiz ) + Posiy * cos( tmp_Posiz );
	gPosiz += Posiz;
	
	double refVx, refVy, refVz;
	
	// ベジエ曲線
	if( phase < 2 ){
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
		
		double onx, ony;    //ベジエ曲線上の点
        onx = bezier_x(phase, t_be);
        ony = bezier_y(phase, t_be);
		
		// 外積による距離導出
        double angle;
        angle = atan2(dbezier_y(phase, t_be), dbezier_x(phase, t_be)); // ベジエ曲線の接線方向
        double dist;
        dist = (ony - gPosiy)*cos(angle) - (onx - gPosix)*sin(angle);
		
		double refVtan, refVper, refKakudo, refVrot;
        refVtan = sokduo_filter.SecondOrderLag(refvel[phase]);
        refVper = yokozurePID.getCmd(dist, 0.0, refvel[phase]);
        //refKakudo = kakudo_filter.SecondOrderLag(refangle[phase]);
        refVrot = kakudoPID.getCmd(angle, gPosiz, 1.57);//(refKakudo, gPosiz, 1.57);
		
		// グローバル座標系の指令速度
        // refVxg = refVtan * cos(angle) - refVper * sin(angle);
        // refVyg = refVtan * sin(angle) + refVper * cos(angle);
        // refVzg = refVrot;
        
        // ローカル座標系の指令速度
        // refVx =  refVxg * cos(gPosiz) + refVyg * sin(gPosiz);
        // refVy = -refVxg * sin(gPosiz) + refVyg * cos(gPosiz);
        // refVz =  refVzg;
        
		// ローカル座標系の指令速度(グローバル座標系のも込み込み)
		refVx =  refVtan * cos( gPosiz - angle ) + refVper * sin( gPosiz - angle );
		refVy = -refVtan * sin( gPosiz - angle ) + refVper * cos( gPosiz - angle );
		refVz = refVrot;
		
        double syusoku;
        syusoku = sqrt(pow(gPosix-Px[3*phase+3], 2.0) + pow(gPosiy-Py[3*phase+3], 2.0));
        if(syusoku <= 0.05 || t_be >= 0.997){
            phase++;
            pre_t_be = 0.1;
        }
        
        epsilon = 1.0;
	} else {
		// PIDクラスを使って位置制御を行う(速度の指令地を得る)
        refVxg = posiPIDx.getCmd(Px[6], gPosix, refvel[phase]);//(Px[21], gPosix, refvel[phase]);
        refVyg = posiPIDy.getCmd(Py[6], gPosiy, refvel[phase]);//(Py[21], gPosiy, refvel[phase]);
        refVzg = posiPIDz.getCmd(0.0, gPosiz, refvel[phase]);

        // 上記はグローバル座標系における速度のため，ローカルに変換
        refVx =  refVxg * cos(gPosiz) + refVyg * sin(gPosiz);
        refVy = -refVxg * sin(gPosiz) + refVyg * cos(gPosiz);
        refVz =  refVzg;
	}
	
	// ローカル速度から，各車輪の角速度を計算
	double refOmegaA, refOmegaB, refOmegaC, refOmegaD;
	refOmegaA = ( refVx - refVy -refVz * ( MECANUM_HANKEI_D + MECANUM_HANKEI_L ) ) / MECANUM_HANKEI;
	refOmegaB = ( refVx + refVy -refVz * ( MECANUM_HANKEI_D + MECANUM_HANKEI_L ) ) / MECANUM_HANKEI;
	refOmegaC = ( refVx - refVy +refVz * ( MECANUM_HANKEI_D + MECANUM_HANKEI_L ) ) / MECANUM_HANKEI;
	refOmegaD = ( refVx + refVy +refVz * ( MECANUM_HANKEI_D + MECANUM_HANKEI_L ) ) / MECANUM_HANKEI;
	
	// 速度制御のためのコマンドをPIDクラスから得る
	// 最大値を超えていた場合に制限をかける
	
	// モータにcmd?を送り，回す
	// MD.SpeedM1(ADR_MD1, (int)(duty[3])*M_CMD);
	// MD.SpeedM2(ADR_MD1, (int)(duty[2])*M_CMD);
	// MD.SpeedM1(ADR_MD2, (int)(duty[1])*M_CMD);
	// MD.SpeedM2(ADR_MD2, (int)(duty[0])*M_CMD);
	
	pre_EncountA = EncountA;
	pre_EncountB = EncountB;
	pre_EncountC = EncountC;
	
	
	/* Serial.print( "MTU1:" );
	Serial.print( EncountA );
	Serial.print( "\t" );
	Serial.print( "MTU2:" );
	Serial.print( EncountB );
	Serial.print( "\t" );
	Serial.print( "TPU1:" );
	Serial.println( EncountC ); */
	Serial.print( gPosix );
	Serial.print( "\t" );
	Serial.print( gPosiy );
	Serial.print( "\t" );
	Serial.println( gPosiz );
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
	
	// RoboClaw
	MD.begin(115200);
	
	// PCと通信
	Serial.begin(115200);
	
	// PID関連初期化
	velPIDA.PIDinit(0.0, 0.0);
	velPIDB.PIDinit(0.0, 0.0);
	velPIDC.PIDinit(0.0, 0.0);
	
	posiPIDx.PIDinit(0.0, 0.0);
	posiPIDy.PIDinit(0.0, 0.0);
	posiPIDz.PIDinit(0.0, 0.0);
	
	yokozurePID.PIDinit(0.0, 0.0);
	kakudoPID.PIDinit(0.0, 0.0);
	
	for(int i = 0; i < 7; i++) {
        Ax[i] = Px[3*i+3] -3*Px[3*i+2] + 3*Px[3*i+1] - Px[3*i+0];
        Ay[i] = Py[3*i+3] -3*Py[3*i+2] + 3*Py[3*i+1] - Py[3*i+0];
        Bx[i] = Px[3*i+2] -2*Px[3*i+1] + Px[3*i+0];
        By[i] = Py[3*i+2] -2*Py[3*i+1] + Py[3*i+0];
        Cx[i] = Px[3*i+1] - Px[3*i+0];
        Cy[i] = Py[3*i+1] - Py[3*i+0];
        Dx[i] = Px[3*i+0];
        Dy[i] = Py[3*i+0];
    }

    for(int i = 0; i < 7; i++) {
        a_be[i] = pow(Ax[i], 2.0) + pow(Ay[i], 2.0);
        b_be[i] = 5*(Ax[i]*Bx[i] + Ay[i]*By[i]);
        c_be[i] = 2*((3*pow(Bx[i],2.0)+2*Ax[i]*Cx[i]) + (3*pow(By[i],2.0)+2*Ay[i]*Cy[i]));
        d_be_[i] = 9*Bx[i]*Cx[i] + 9*By[i]*Cy[i];
        e_be_[i] = 3*pow(Cx[i],2.0) + 3*pow(Cy[i],2.0);
        f_be_[i] = 0;
    }
	
	// タイマー割り込み(とりあえず10ms)
	MsTimerTPU3::set((int)(INT_TIME * 1000), timer_warikomi); // 10ms period
	MsTimerTPU3::start();
}


// reboot用関数
void reboot_function(){
	system_reboot(REBOOT_USERAPP);
}


void loop() {
	if(!digitalRead(PIN_SW)){
		reboot_function();
	}
}
