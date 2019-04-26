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
RoboClaw MD(&Serial2,5);//10);

PID posiPIDx(7.0, 0.0, 0.0, INT_TIME);
PID posiPIDy(6.0, 0.0, 0.0, INT_TIME);
PID posiPIDz(9.0, 0.0, 0.0, INT_TIME);

PID yokozurePID(1.5, 0.0, 0.0, INT_TIME);
PID kakudoPID(3.0, 0.0, 0.0, INT_TIME);

// 二次遅れ使えるようになる
Filter sokduo_filter(INT_TIME);
Filter kakudo_filter(INT_TIME);

// ベジエ曲線用
double Px[22] = /* P0が頭 */
	/* 0 */{ 0.50, 0.50, 1.515,
	/* 1 */1.525, 1.525, 0.925, 
	/* 2 */0.925, 0.925, 1.525, 
	/* 3 */1.525, 1.525, 1.225, 
	/* 4 */1.225, 1.225, 1.225, 
	/* 5 */1.225, 1.225, 3.0, 
	/* 6 */4.0, 5.8, 6.0, 
	/* 7 */6.0 };
//{ 0.50, 0.50, 1.94, 1.94, 1.94, 0.51, 0.51 };
double Py[22] = 
	/* 0 */{ 0.50, 1.50, 1.00, 
	/* 1 */2.00, 3.00, 2.50, 
	/* 2 */3.50, 4.50, 4.00, 
	/* 3 */5.00, 6.00, 5.75, 
	/* 4 */6.50, 7.25, 7.50, 
	/* 5 */8.00, 10.0, 8.30, 
	/* 6 */8.30, 8.30, 8.30, 
	/* 7 */8.30 };//{ 0.50, 1.25, 1.25, 2.00, 2.75, 2.75, 3.50 };

double refvel[7] = {0.9,0.9,0.9,0.9,0.9,0.9,0.9};//{1.2,1.2,1.2,1.2,1.2,1.2,1.2};//{0.5,0.5,0.5,0.5,0.5,0.5,0.5};

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
int pre_tmpEncA = 0, pre_tmpEncB = 0, pre_tmpEncC = 0;
int ledcount = 0;

int tenCount = 9;//0;
boolean tenFlag = false;

int data1[ 1300 ];//[ 12000 ];
int data2[ 1300 ];
int data3[ 1300 ];
int data4[ 1300 ];

int *pdata1 = data1;
int *pdata2 = data2;
int *pdata3 = data3;
int *pdata4 = data4;

// グローバル変数の設定
double gPosix = Px[0], gPosiy = Py[0], gPosiz = 1.57080;//0;


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
    count++;
	
	if(ledcount >= 100) {
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
    EncountC =  Enc3.getCount();	// TPU1, xr
	
	// 角度   encountはdoubleに型変換した方がいいかもしれない
	double Kakudoxl, Kakudoxr, Kakudoy;
	Kakudoxl = ( double )( EncountA - pre_EncountA ) * _2PI_MEASRMX;
	Kakudoxr = ( double )( EncountC - pre_EncountC ) * _2PI_MEASRMX;
	Kakudoy  = ( double )( EncountB - pre_EncountB ) * _2PI_MEASRMY;
	
	// ローカル用(zは角度)
	double Posix, Posiy, Posiz;
	double Posixl, Posixr;
	Posiz = ( MEASURE_HANKEI_X_R * Kakudoxr - MEASURE_HANKEI_X_L * Kakudoxl ) * _0P5_MEASHD;
	Posix = ( MEASURE_HANKEI_X_L * Kakudoxl + MEASURE_HANKEI_X_R * Kakudoxr ) * 0.5;
	Posixl = MEASURE_HANKEI_X_L * Kakudoxl;
	Posixr = MEASURE_HANKEI_X_R * Kakudoxr;
	Posiy = MEASURE_HANKEI_Y * Kakudoy - MEASURE_HANKEI_L * Posiz;
	
	// static double tmpPosix = 0.0, tmpPosixl = 0.0, tmpPosixr = 0.0, tmpPosiy = 0.0, tmpPosiz = 0.0;
	// tmpPosix += Posix;
	// tmpPosixl += Posixl;
	// tmpPosixr += Posixr;
	// tmpPosiy += Posiy;
	// tmpPosiz += Posiz;

	// グローバル用(zは角度)
	double tmp_Posiz = gPosiz + ( Posiz * 0.5 ); // つまりgPosi + ( Posiz / 2.0 );
	gPosix += Posix * cos( tmp_Posiz ) - Posiy * sin( tmp_Posiz );
	gPosiy += Posix * sin( tmp_Posiz ) + Posiy * cos( tmp_Posiz );
	gPosiz += Posiz;
	
	

	tenCount++;

	if( tenCount == 10 ){
		tenFlag = true;
		tenCount = 0;
	}

	

	pre_EncountA = EncountA;
	pre_EncountB = EncountB;
	pre_EncountC = EncountC;

	// pre_tmpEncA = tmpEncA;
	// pre_tmpEncB = tmpEncB;
	// pre_tmpEncC = tmpEncC;
	
	
	/* Serial.print( "refX:" );
	Serial.print( "\t" );
	Serial.print( gPosiz , 4);
	Serial.print( "\t" );
	Serial.print( "vel:" );
	Serial.print( "\t" );
	Serial.println( refVzg , 4 ); */
	/* Serial.print( gPosix );
	Serial.print( "\t" );
	Serial.print( gPosiy );
	Serial.print( "\t" );
	Serial.println( gPosiz ); */
	/* Serial.print( "k_L:" );
	Serial.print( tmpKakudoxl, 4 );
	Serial.print( "\t" );
	Serial.print( "k_R:" );
	Serial.print( tmpKakudoxr, 4 ); */
	//Serial.print( "b_x:" );
	//Serial.print( onx, 2 );
	//Serial.print( "\t" );
	//Serial.print( "b_y:" );
	//Serial.print( ony, 2 );
	//Serial.print( "\t" );
	/* Serial.print( "v_x:" );
	Serial.print( refVx, 2 );
	Serial.print( ", " );
	Serial.print( "v_y:" );
	Serial.print( refVy, 2 );
	Serial.print( ", " );
	Serial.print( "v_z:" );
	Serial.print( refVz, 2 );
	Serial.print( ", " ); */
	//Serial.print( "g_x:" );
	//Serial.print( gPosix, 2 );
	//Serial.print( "\t" );
	//Serial.print( "g_y:" );
	//Serial.print( gPosiy, 2 );
	//Serial.print( "\t" );
	//Serial.print( "g_z:" );
	//Serial.println(phase);//( gPosiz, 2 );


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
	Serial.begin(115200);
	
	// PID関連初期化
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

	if( tenFlag ){
		static int phase = 0;
		double refVx, refVy, refVz;
	
		double onx, ony;    //ベジエ曲線上の点

		// ベジエ曲線
		if( phase < 7 ){
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
			double angle;
			angle = atan2(dbezier_y(phase, t_be), dbezier_x(phase, t_be)); // ベジエ曲線の接線方向
			double dist;
			dist = (ony - gPosiy)*cos(angle) - (onx - gPosix)*sin(angle);
			
			double refVtan, refVper, refVrot;
			refVtan = sokduo_filter.SecondOrderLag(refvel[phase]);
			refVper = yokozurePID.getCmd(dist, 0.0, refvel[phase]);
			//refKakudo = kakudo_filter.SecondOrderLag(refangle[phase]);
			refVrot = kakudoPID.getCmd(angle, gPosiz, 1.57);//(refKakudo, gPosiz, 1.57);
			
			// ローカル座標系の指令速度(グローバル座標系のも込み込み)
			refVx =  refVtan * cos( gPosiz - angle ) + refVper * sin( gPosiz - angle );
			refVy = -refVtan * sin( gPosiz - angle ) + refVper * cos( gPosiz - angle );
			refVz = refVrot;//0.628319;だと10秒で旋回？
			
			double syusoku;
			syusoku = sqrt(pow(gPosix-Px[3*phase+3], 2.0) + pow(gPosiy-Py[3*phase+3], 2.0));
			if(syusoku <= 0.05 || t_be >= 0.997){
				phase++;
				pre_t_be = 0.1;
			}
			
			epsilon = 1.0;
		} else {
			// PIDクラスを使って位置制御を行う(速度の指令地を得る)
			refVxg = posiPIDx.getCmd(Px[21], gPosix, refvel[phase]);
			refVyg = posiPIDy.getCmd(Py[21], gPosiy, refvel[phase]);
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
		MD.SpeedM1(ADR_MD1,  (int)mdCmdB);// 左後
		MD.SpeedM2(ADR_MD1, -(int)mdCmdC);// 右後
		MD.SpeedM1(ADR_MD2,  (int)mdCmdA);// 左前
		MD.SpeedM2(ADR_MD2, -(int)mdCmdD);// 右前

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

		

		if( dataCount < 1200){//11990 ){
			*(pdata1 + dataCount) = ( int )( onx * 1000 );
			//dataCount++;
			*(pdata2 + dataCount) = ( int )( ony * 1000 );
			//dataCount++;
			*(pdata3 + dataCount) = ( int )( gPosix * 1000 );
			//dataCount++;
			*(pdata4 + dataCount) = ( int )( gPosiy * 1000 );
			dataCount++;
			/* *(pdata + dataCount) = ( int )( ony * 1000 );
			dataCount++; */
			/* *(pdata + dataCount) = ( int )( gPosix * 1000 );
			dataCount++;
			*(pdata + dataCount) = ( int )( gPosiy * 1000 );
			dataCount++; */
		}else{
			//dataFlag = 1;
		}



		if( !digitalRead(PIN_SW) ){//if( dataFlag && !dataend ){
			for( int forcount = 0; forcount < 1200 ; forcount++ ){
				//Serial.print(forcount);
				//Serial.print("\t");
				Serial.print( *(pdata1 + forcount) );
				Serial.print("\t");
				Serial.print( *(pdata2 + forcount) );
				Serial.print("\t");
				Serial.print( *(pdata3 + forcount) );
				Serial.print("\t");
				Serial.println( *(pdata4 + forcount) );
				if( forcount == 1199 ){
					dataend = true;
				}
			}
		}

		tenFlag = false;
	}
}
