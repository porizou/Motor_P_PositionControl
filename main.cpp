#include "mbed.h"
#include "rtos.h"

#include "Encoder.h"

#include "motordriver.h"

#define V_CONTROL_CYCLE 0.01 //速度制御周期[s]
#define P_CONTROL_CYCLE 0.1 //位置制御周期[s]

#define Kp 0.03 //速度比例ゲイン
#define Ki 0.578 //速度積分ゲイン

#define Kpp 2.0 //位置比例ゲイン

#define Vmax 8.2 //モータの電源電圧[V]
#define OMEGA_MAX 400//モータの最大回転速度[rad/s]

DigitalOut led(LED1);



Serial pc(USBTX, USBRX); // tx, rx

Encoder Enc(p21, p22, 60, 0.001, 0.01);
Motor motor(p25 , p23 , p24 , 10000);

float accError = 0.0; //偏差の積分値
float prevError = 0.0; //前回の偏差

float Pref = 0.0; //位置の目標値[rad]

float Vref = 0.0; //速度の目標値[rad/s]

/* 位置制御スレッド */
void PositionThread(void const *argument) {
    float P = Enc.getAngle();//角度の値を取得[rad/s]
    
    float Error = Pref - P;//偏差の計算
    
    
    Vref = Error * Kpp; //出力速度の計算
    if(Vref > OMEGA_MAX) Vref = OMEGA_MAX;
    if(Vref < -OMEGA_MAX) Vref = -OMEGA_MAX;
}


/* 速度制御スレッド */
void VelocityThread(void const *argument) {

    float V = Enc.getVelocity(); //速度の値を取得[rad/s]
    
    float Error = Vref - V; //偏差の計算
    
    accError += (Error + prevError) / 2.0 * V_CONTROL_CYCLE; //偏差の積分値の計算
    
    float Vout = Kp * Error + Ki * accError; //出力電圧を計算
    
    motor.rotate(Vout / Vmax); //モータドライバに出力
    
    prevError = Error; //偏差の更新
}

int main() {
    
    //pc.baud(115200);
    
    RtosTimer Velocity(VelocityThread); //速度制御
    RtosTimer Position(PositionThread); //位置制御
    
    Velocity.start(V_CONTROL_CYCLE * 1000);
    Position.start(P_CONTROL_CYCLE * 1000);
    //motor.rotate(1.0);
 
    while(1) {
        Pref = 2*PI / 4 * 42;
        wait(1.0);
        Pref = 0.0;
        wait(1.0);
    }
}
