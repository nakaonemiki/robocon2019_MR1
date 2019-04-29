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
double Px[31] = /* P0が頭 */
	/* 0 */{ 0.500, 1.000, 1.225,
	/* 1 */1.475, 1.725, 1.725, 
	/* 2 */1.475, 1.225, 1.165, 
	/* 3 */0.945, 0.725, 0.725, 
	/* 4 */0.945, 1.045, 1.225, 
	/* 5 */1.475, 1.725, 1.600, 
	/* 6 */1.450, 1.300, 1.050, 
	/* 7 */1.225, 1.400, 1.800,
	/* 8 */2.300, 2.800, 3.500,
	/* 9 */4.000, 5.000, 5.500,
	/* 10 */6.050 };
//{ 0.50, 0.50, 1.94, 1.94, 1.94, 0.51, 0.51 };
double Py[31] = 
	/* 0 */{ 0.500, 1.038, 1.281, 
	/* 1 */1.550, 1.819, 2.167, 
	/* 2 */2.450, 2.733, 2.801, 
	/* 3 */3.050, 3.299, 3.722, 
	/* 4 */3.950, 4.054, 4.241, 
	/* 5 */4.500, 4.759, 5.400, 
	/* 6 */5.800, 6.200, 7.850, 
	/* 7 */8.200, 8.550, 8.572,
	/* 8 */8.500, 8.427, 8.255,
	/* 9 */8.255, 8.255, 8.255,
	/* 10 */8.255 };

double refvel[10] = {/*A*/0.4,/*B*/0.4,/*C*/0.4,/*D*/0.4,/*E*/0.4,/*F*/0.4,/*G*/0.4,/*H*/0.4,/*I*/0.4,/*J*/0.4};//{0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5};//{0.9,0.9,0.9,0.9,0.9,0.9,0.9};//{1.2,1.2,1.2,1.2,1.2,1.2,1.2};//

// ベジエ曲線関連
double Ax[10];
double Bx[10];
double Cx[10];
double Dx[10];

double Ay[10];
double By[10];
double Cy[10];
double Dy[10];

// 内積関連
double a_be[10];
double b_be[10];
double c_be[10];
double d_be[10];
double e_be[10];
double f_be[10];
double d_be_[10];
double e_be_[10];
double f_be_[10];

double t_be = 0.0;
double pre_t_be = 0.1;
double epsilon = 1.0;

double refVxg, refVyg, refVzg; // グローバル座標系の指定速度
int EncountA = 0, EncountB = 0, EncountC = 0;
int pre_EncountA = 0, pre_EncountB = 0, preAngleC = 0;
int pre_tmpEncA = 0, pre_tmpEncB = 0, pre_tmpEncC = 0;
int ledcount = 0;

int tenCount = 0;//0;
boolean tenFlag = false;

int data1[ 1300 ];//[ 12000 ];
int data2[ 1300 ];
int data3[ 1300 ];
int data4[ 1300 ];

int *pdata1 = data1;
int *pdata2 = data2;
int *pdata3 = data3;
int *pdata4 = data4;

// R1370
int state, counter;
byte buffer[15];

/* double Kakudoxl, Kakudoxr, Kakudoy, tmpKakudoy;
double Posix, Posiy, Posiz;
double Posixl, Posixr;
double tmpPosix = 0.0, tmpPosixl = 0.0, tmpPosixr = 0.0, tmpPosiy = 0.0, tmpPosiz = 0.0; */


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
    EncountA = -Enc1.getCount();//-Enc1.getCount();	// MTU1, xl
    EncountB = -Enc2.getCount();//-Enc2.getCount();	// MTU2, y
    EncountC =  Enc3.getCount();// Enc3.getCount();	// TPU1, xr
	
	// 角度   encountはdoubleに型変換した方がいいかもしれない
	double Kakudoxl, Kakudoxr, Kakudoy;
	Kakudoxl = ( double )( EncountA - pre_EncountA ) * _2PI_MEASRMX;
	Kakudoxr = ( double )( EncountC - pre_EncountC ) * _2PI_MEASRMX;
	Kakudoy  = ( double )( EncountB - pre_EncountB ) * _2PI_MEASRMY;
	
	while( !recv_done ){
		if (Serial3.available() > 0) {
			byte data = Serial3.read();
			switch (state) {
			case 0:
				if (data == 0xaa) {
					state++;
				}
				break;
			case 1:
				if (data == 0x00) {
					state++;
				} 
				else {
					counter = 0;
					state = 0;
				}
				break;
			case 2:
				buffer[counter++] = data;
				if (counter >= 13) {
					int sum = 0;
					for (int i = 0; i < 11; i++) sum += buffer[i];
					if ((sum & 0xff) == buffer[12]) {
						//Serial.print("A:");
						rawangle = (short)(buffer[1] | (buffer[2] << 8)) / 100.0;
						if(fabs(rawangle - pre_rawangle) >= 100){
							if(rawangle < 0){ //+から-へ回ったとき 
								angle_deg += 360 + rawangle - pre_rawangle;
							}else{ // -から+へ回ったとき
								angle_deg += -360 + rawangle -pre_rawangle;
							}
						}else{
							angle_deg += rawangle - pre_rawangle;
						}
						// degからradに
						angle_rad = angle_deg * ( PI / 180.0 );

						pre_rawangle = rawangle;

						Serial.println(angle_rad);
						//Serial.println((buffer[2] << 8) | buffer[1]);  // angle
						//Serial.print("\tX:");
						//Serial.print((buffer[6] << 8) | buffer[5]);  // accX
						//Serial.print("\tY:");
						//Serial.print((buffer[8] << 8) | buffer[7]);  // accY
						//Serial.print("\tZ:");
						//Serial.println((buffer[10] << 8) | buffer[9]);  // accZ
						//delay(10);
						Serial3.flush();
					}
					state = 0;
					counter = 0;
					recv_done = true;
				}
				break;
			}
		}
	}
	recv_done = false;

	// tmpKakudoy += Kakudoy;

	// ローカル用(zは角度)
	double Posix, Posiy, Posiz;
	double Posixl, Posixr;
	static double pre_angle_rad = angle_rad;
	double angle_diff;
	angle_diff = angle_rad - pre_angle_rad;
	Posiz = angle_diff;//( MEASURE_HANKEI_X_R * Kakudoxr - MEASURE_HANKEI_X_L * Kakudoxl ) * _0P5_MEASHD;//
	Posix = ( MEASURE_HANKEI_X_L * Kakudoxl + MEASURE_HANKEI_X_R * Kakudoxr ) * 0.5;
	Posixl = MEASURE_HANKEI_X_L * Kakudoxl;
	Posixr = MEASURE_HANKEI_X_R * Kakudoxr;
	Posiy = MEASURE_HANKEI_Y * Kakudoy - MEASURE_HANKEI_L * Posiz;
	
	static double tmpPosix = 0.0, tmpPosixl = 0.0, tmpPosixr = 0.0, tmpPosiy = 0.0, tmpPosiz = 0.0;
	tmpPosix += Posix;
	tmpPosixl += Posixl;
	tmpPosixr += Posixr;
	tmpPosiy += Posiy;
	tmpPosiz += Posiz;

	// グローバル用(zは角度)
	double tmp_Posiz = gPosiz + ( Posiz * 0.5 ); // つまりgPosi + ( Posiz / 2.0 );
	gPosix += Posix * cos( gPosiz ) - Posiy * sin( gPosiz );//Posix * cos( tmp_Posiz ) - Posiy * sin( tmp_Posiz );
	gPosiy += Posix * sin( gPosiz ) + Posiy * cos( gPosiz );//Posix * sin( tmp_Posiz ) + Posiy * cos( tmp_Posiz );
	gPosiz += Posiz;
	
	tenCount++;

	if( tenCount == 1 ){
		tenFlag = true;
		tenCount = 0;
	}

	pre_EncountA = EncountA;
	pre_EncountB = EncountB;
	pre_EncountC = EncountC;

	pre_angle_rad = angle_rad;

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

	// R1370
	Serial3.begin(115200);
	state = 0;
	counter = 0;
	
	// PID関連初期化
	posiPIDx.PIDinit(0.0, 0.0);
	posiPIDy.PIDinit(0.0, 0.0);
	posiPIDz.PIDinit(0.0, 0.0);
	
	yokozurePID.PIDinit(0.0, 0.0);
	kakudoPID.PIDinit(0.0, 0.0);
	
	for(int i = 0; i < 10; i++) {
        Ax[i] = Px[3*i+3] -3*Px[3*i+2] + 3*Px[3*i+1] - Px[3*i+0];
        Ay[i] = Py[3*i+3] -3*Py[3*i+2] + 3*Py[3*i+1] - Py[3*i+0];
        Bx[i] = Px[3*i+2] -2*Px[3*i+1] + Px[3*i+0];
        By[i] = Py[3*i+2] -2*Py[3*i+1] + Py[3*i+0];
        Cx[i] = Px[3*i+1] - Px[3*i+0];
        Cy[i] = Py[3*i+1] - Py[3*i+0];
        Dx[i] = Px[3*i+0];
        Dy[i] = Py[3*i+0];
    }

    for(int i = 0; i < 10; i++) {
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

	
	
	//while( counter < 13 ){
	

	if( tenFlag ){		
		static int phase = 0;
		double refVx, refVy, refVz;

		double onx, ony;    //ベジエ曲線上の点

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
			refVz =  refVrot;//0.628319;だと10秒で旋回？
			
			double syusoku;
			syusoku = sqrt(pow(gPosix-Px[3*phase+3], 2.0) + pow(gPosiy-Py[3*phase+3], 2.0));
			if(syusoku <= 0.02 || t_be >= 0.997){//(syusoku <= 0.05 || t_be >= 0.997){
				Px[3*phase+3] = gPosix;
				Py[3*phase+3] = gPosiy;
				phase++;
				pre_t_be = 0.1;
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
		MD.SpeedM1(ADR_MD1, -(int)mdCmdB);// (int)mdCmdB);// 左後
		MD.SpeedM2(ADR_MD1,  (int)mdCmdC);//-(int)mdCmdC);// 右後
		MD.SpeedM1(ADR_MD2, -(int)mdCmdA);// (int)mdCmdA);// 左前
		MD.SpeedM2(ADR_MD2,  (int)mdCmdD);//-(int)mdCmdD);// 右前

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

		

		/* if( dataCount < 1200){//11990 ){
			*(pdata1 + dataCount) = ( int )( onx * 1000 );
			*(pdata2 + dataCount) = ( int )( ony * 1000 );
			*(pdata3 + dataCount) = ( int )( gPosix * 1000 );
			*(pdata4 + dataCount) = ( int )( gPosiy * 1000 );
			dataCount++;
		}else{
			//dataFlag = 1;
		} */



		/* if( !digitalRead(PIN_SW) ){//if( dataFlag && !dataend ){
			for( int forcount = 0; forcount < 1200 ; forcount++ ){
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
		} */

		/* Serial.print(onx, 4);//xl
		Serial.print("\t");
		Serial.print(ony, 4);//y
		Serial.print("\t");
		Serial.print(gPosix, 4);
		Serial.print("\t");
		Serial.print(gPosiy, 4);
		Serial.print("\t");
		Serial.println(gPosiz, 4); */

		/* Serial.print(gPosix, 4);
		Serial.print("\t");
		Serial.print(gPosiy, 4);
		Serial.print("\t");
		Serial.println(gPosiz, 4); */

		tenFlag = false;
	}
}
