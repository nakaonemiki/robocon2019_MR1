//-----------------------------------------
// ベジエ曲線に追従するときに必要になるベジエ曲線までのズレを計算するクラス
// 2019/05/15 ueno
//-----------------------------------------

#include "MotionGenerator.h"
#include "PIDclass.h"
#include "Filter.h"
#include "define.h"

PID posiPIDx(5.0, 0.0, 0.0, INT_TIME);
PID posiPIDy(4.0, 0.0, 0.0, INT_TIME);
PID posiPIDz(5.0, 0.0, 0.0, INT_TIME);

PID yokozurePID(3.0, 0.0, 0.0, INT_TIME);
PID kakudoPID(3.0, 0.0, 0.0, INT_TIME);

// 二次遅れ使えるようになる
Filter sokduo_filter(INT_TIME);
Filter kakudo_filter(INT_TIME);

// コンストラクタ
MotionGenerator::MotionGenerator(int xmode){
    path_num = 0;
    t_be = 0.0;
    pre_t_be = 0.1;
    epsilon = 1.0;

    refVx = 0;
    refVy = 0;
    refVz = 0;

    mode = xmode;

    // PID関連初期化
	posiPIDx.PIDinit(0.0, 0.0);
	posiPIDy.PIDinit(0.0, 0.0);
	posiPIDz.PIDinit(0.0, 0.0);
	
	yokozurePID.PIDinit(0.0, 0.0);
	kakudoPID.PIDinit(0.0, 0.0);

	sokduo_filter.setSecondOrderPara(15.0, 1.0, 0.0);
    kakudo_filter.setSecondOrderPara(7.0, 1.0, 0.0);

    mode_changed = true;
    init_done = false;
}

// tを求めるための方程式
double MotionGenerator::func(int p, double t)
{
    return a_be[p] * pow(t, 5.0) + b_be[p] * pow(t,4.0) + c_be[p] * pow(t,3.0) + d_be[p] * pow(t,2.0) + e_be[p] * t + f_be[p];
}
// tを求めるための方程式の1階微分
double MotionGenerator::dfunc(int p, double t)
{
    return 5.0 * a_be[p] * pow(t, 4.0) +  4.0 * b_be[p] * pow(t,3.0) + 3.0 * c_be[p] * pow(t,2.0) + 2.0 * d_be[p] * t + e_be[p];
}

// tにおけるベジエ曲線の座標を求める関数
double MotionGenerator::bezier_x(int p, double t)
{
    return Ax[p]*pow(t,3.0) + 3.0*Bx[p]*pow(t,2.0) + 3.0*Cx[p]*t + Dx[p];
}
double MotionGenerator::bezier_y(int p, double t)
{
    return Ay[p]*pow(t,3.0) + 3.0*By[p]*pow(t,2.0) + 3.0*Cy[p]*t + Dy[p];
}

// ベジエ曲線式の1階微分
double MotionGenerator::dbezier_x(int p, double t)
{
    return 3.0*Ax[p]*pow(t,2.0) + 6.0*Bx[p]*t + 3.0*Cx[p];
}
double MotionGenerator::dbezier_y(int p, double t)
{
    return 3.0*Ay[p]*pow(t,2.0) + 6.0*By[p]*t + 3.0*Cy[p];
}

// ニュートン法のための係数の初期化
void MotionGenerator::initSettings(){
    for(int i = 0; i < PATHNUM; i++) {
        Ax[i] = Px[3*i+3] -3*Px[3*i+2] + 3*Px[3*i+1] - Px[3*i+0];
        Ay[i] = Py[3*i+3] -3*Py[3*i+2] + 3*Py[3*i+1] - Py[3*i+0];
        Bx[i] = Px[3*i+2] -2*Px[3*i+1] + Px[3*i+0];
        By[i] = Py[3*i+2] -2*Py[3*i+1] + Py[3*i+0];
        Cx[i] = Px[3*i+1] - Px[3*i+0];
        Cy[i] = Py[3*i+1] - Py[3*i+0];
        Dx[i] = Px[3*i+0];
        Dy[i] = Py[3*i+0];
    }

    for(int i = 0; i < PATHNUM; i++) {
        a_be[i] = pow(Ax[i], 2.0) + pow(Ay[i], 2.0);
        b_be[i] = 5*(Ax[i]*Bx[i] + Ay[i]*By[i]);
        c_be[i] = 2*((3*pow(Bx[i],2.0)+2*Ax[i]*Cx[i]) + (3*pow(By[i],2.0)+2*Ay[i]*Cy[i]));
        d_be_[i] = 9*Bx[i]*Cx[i] + 9*By[i]*Cy[i];
        e_be_[i] = 3*pow(Cx[i],2.0) + 3*pow(Cy[i],2.0);
        f_be_[i] = 0;
    }
    init_done = true;
}

// ベジエ曲線までの垂線距離をニュートン法で求めて，そこまでの距離と接線角度を計算する
void MotionGenerator::calcRefpoint(double Posix, double Posiy){
    if(init_done){
        double tmpx = Px[path_num * 3] - Posix;
        double tmpy = Py[path_num * 3] - Posiy;
                
        d_be[path_num] = d_be_[path_num] + Ax[path_num] * tmpx + Ay[path_num] * tmpy;
        e_be[path_num] = e_be_[path_num] + 2*Bx[path_num] * tmpx + 2*By[path_num] * tmpy;
        f_be[path_num] = f_be_[path_num] + Cx[path_num] * tmpx + Cy[path_num] * tmpy;
                
        int count_newton = 0;
        do {
            t_be = pre_t_be - func(path_num, pre_t_be)/dfunc(path_num, pre_t_be);
            epsilon = abs((t_be - pre_t_be)/pre_t_be);
            
            pre_t_be = t_be;
            count_newton++;
        }while(epsilon >= 1e-4 && count_newton <= 50);
        
        //double onx, ony;    //ベジエ曲線上の点
        onx = bezier_x(path_num, t_be);
        ony = bezier_y(path_num, t_be);
                
        // 外積による距離導出
        //double angle;
        angle = atan2(dbezier_y(path_num, t_be), dbezier_x(path_num, t_be)); // ベジエ曲線の接線方向
        //double dist;
        dist = (ony - Posiy)*cos(angle) - (onx - Posix)*sin(angle);

        epsilon = 1.0;
    }
}

// モードによって，それぞれ指令速度を計算する
int MotionGenerator::calcRefvel(double Posix, double Posiy, double Posiz){
    double refVxg, refVyg, refVzg; // グローバル座標系の指定速度

    if(init_done){
        if(mode == FOLLOW_TANGENT || mode == FOLLOW_COMMAND){ // ベジエ曲線追従モード
            calcRefpoint(Posix, Posiy);

            double refVtan, refVper, refVrot;
            refVtan = sokduo_filter.SecondOrderLag(refvel[path_num]); // 接線方向速度
            refVper = yokozurePID.getCmd(dist, 0.0, refvel[path_num]); // 横方向速度

            // 旋回は以下の2種類を mode によって変える
            if(mode == FOLLOW_TANGENT){
                if(mode_changed){
                    kakudoPID.PIDinit(angle, Posiz);
                    mode_changed = false;
                }
                refVrot = kakudoPID.getCmd(angle, Posiz, 1.57);//(refKakudo, gPosiz, 1.57);
            }else{
                if(mode_changed){
                    kakudoPID.PIDinit(refKakudo, Posiz);
                    kakudo_filter.initPrevData(refKakudo);
                    mode_changed = false;
                }
                refKakudo = kakudo_filter.SecondOrderLag(refangle[path_num]);
                refVrot = kakudoPID.getCmd(refKakudo, Posiz, 1.57);
            }

            // ローカル座標系の指令速度(グローバル座標系のも込み込み)
            refVx =  refVtan * cos( Posiz - angle ) + refVper * sin( Posiz - angle );
            refVy = -refVtan * sin( Posiz - angle ) + refVper * cos( Posiz - angle );
            refVz =  refVrot;//0.628319;だと10秒で旋回？
        }else{ // PID位置制御モード
            if( mode_changed ){
				posiPIDx.PIDinit(Px[3 * path_num], Posix);	// ref, act
				posiPIDy.PIDinit(Py[3 * path_num], Posiy);
				posiPIDz.PIDinit(refangle[path_num], Posiz);
                kakudo_filter.initPrevData(refKakudo);
				mode_changed = false;
			}

			// PIDクラスを使って位置制御を行う(速度の指令地を得る)
			refVxg = posiPIDx.getCmd(Px[3 * path_num], Posix, refvel[path_num]);//(Px[30], gPosix, refvel[phase]);
			refVyg = posiPIDy.getCmd(Py[3 * path_num], Posiy, refvel[path_num]);//(Py[30], gPosiy, refvel[phase]);
            refKakudo = kakudo_filter.SecondOrderLag(refangle[path_num]);
			refVzg = posiPIDz.getCmd(refangle[path_num], Posiz, refvel[path_num]);//(0.0, gPosiz, refvel[phase]);

			// 上記はグローバル座標系における速度のため，ローカルに変換
			refVx =  refVxg * cos(Posiz) + refVyg * sin(Posiz);
			refVy = -refVxg * sin(Posiz) + refVyg * cos(Posiz);
			refVz =  refVzg;
        }

        // 収束判定して，収束していたら　1　を返す
        dist2goal = sqrt(pow(Posix - Px[3 * path_num + 3], 2.0) + pow(Posiy - Py[3 * path_num + 3], 2.0));
        if(dist2goal <= conv_length || t_be >= conv_tnum) return 1;
        
        // 収束していなかったら　0　を返す
        return 0;
    
    }else{
        // 初期化されていなかったら　-1　を返す
        return -1;
    }
}

// ベジエ曲線のパス番号をインクリメントする
void MotionGenerator::incrPathnum(double xconv_length, double xconv_tnum = 0.997){
    path_num++;
    pre_t_be = 0.1;
    setConvPara(xconv_length, xconv_tnum);
}

int MotionGenerator::getPathNum(){
    return path_num;
}

// 収束判定に用いる距離などをセットする
void MotionGenerator::setConvPara(double xconv_length, double xconv_tnum = 0.997){
    conv_length = xconv_length;
    conv_tnum = xconv_tnum;
}

void MotionGenerator::setMode(int xmode){
    mode = xmode;
    mode_changed = true;
}

int MotionGenerator::getMode(){
    return mode;
}

void MotionGenerator::setPosiPIDxPara(float xKp, float xKi, float xKd){
    posiPIDx.setPara(xKp, xKi, xKd);
}

void MotionGenerator::setPosiPIDyPara(float xKp, float xKi, float xKd){
    posiPIDy.setPara(xKp, xKi, xKd);
}

void MotionGenerator::setPosiPIDzPara(float xKp, float xKi, float xKd){
    posiPIDz.setPara(xKp, xKi, xKd);
}

void MotionGenerator::setYokozurePIDPara(float xKp, float xKi, float xKd){
    yokozurePID.setPara(xKp, xKi, xKd);
}

void MotionGenerator::setKakudoPIDPara(float xKp, float xKi, float xKd){
    kakudoPID.setPara(xKp, xKi, xKd);
}