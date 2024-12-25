#include "UFO_Config.h"

#define UFO_TEST_WHEEL_PWM_PIN1 14  
#define UFO_TEST_WHEEL_PWM_PIN2 27

#define UFO_TEST_WHEEL_W1P1     26
#define UFO_TEST_WHEEL_W1P2     25

#define UFO_TEST_WHEEL_W2P1     33
#define UFO_TEST_WHEEL_W2P2     32

#define UFO_TEST_WHEEL_ROTATE 170

#define UFO_MOTOR_VAL_MAX           ((uint16_t)2000)
#define UFO_MOTOR_VAL_MIN           ((uint16_t)1000)

#define UFO_MOTOR_RES_VAL_MAX       ((uint16_t)(std::pow(2, UFO_PWM_RESOLUTION)/10))           //2^16/10
#define UFO_MOTOR_RES_VAL_MIN       ((uint16_t)(UFO_MOTOR_RES_VAL_MAX/2))                           //2^16/2/10


void TestPinSetup(){
    pinMode(UFO_TEST_WHEEL_PWM_PIN1, OUTPUT);
    pinMode(UFO_TEST_WHEEL_PWM_PIN2, OUTPUT);
    pinMode(UFO_TEST_WHEEL_W1P1,    OUTPUT);
    pinMode(UFO_TEST_WHEEL_W1P2,    OUTPUT);
    pinMode(UFO_TEST_WHEEL_W2P1,    OUTPUT);
    pinMode(UFO_TEST_WHEEL_W2P2,    OUTPUT);

    digitalWrite(UFO_TEST_WHEEL_W1P1, LOW);
    digitalWrite(UFO_TEST_WHEEL_W1P2, LOW);
    digitalWrite(UFO_TEST_WHEEL_W2P1, LOW);
    digitalWrite(UFO_TEST_WHEEL_W2P2, LOW);

    // digitalWrite(UFO_TEST_WHEEL_W1P1, HIGH);
    // digitalWrite(UFO_TEST_WHEEL_W1P2, LOW);
    // // Задаём направление для 2-го мотора
    // digitalWrite(UFO_TEST_WHEEL_W2P1, HIGH);
    // digitalWrite(UFO_TEST_WHEEL_W2P2, LOW);
}


void LeftFront(){
    digitalWrite(UFO_TEST_WHEEL_W2P1, HIGH);
    digitalWrite(UFO_TEST_WHEEL_W2P2, LOW);
}

void LeftBack(){
    digitalWrite(UFO_TEST_WHEEL_W2P1, LOW);
    digitalWrite(UFO_TEST_WHEEL_W2P2, HIGH);
}

void RightFront(){
    digitalWrite(UFO_TEST_WHEEL_W1P1, HIGH);
    digitalWrite(UFO_TEST_WHEEL_W1P2, LOW);
}

void RightBack(){
    digitalWrite(UFO_TEST_WHEEL_W1P1, LOW);
    digitalWrite(UFO_TEST_WHEEL_W1P2, HIGH);
}



void TestPinLoop(int p, int r){

    p = map(p, 1000, 2000, -1000, 1000);
    r = map(r, 1000, 2000, -1000, 1000);
    // Serial.print(p);
    // Serial.print("\t\t");
    // Serial.print(r);
    // Serial.print("\t\t");
    float w = sqrt( p*p + r*r );
    // Serial.println(p);

    float s = p/w;

    if (p < 100)
    {
        // Serial.println("tt");
        analogWrite(UFO_TEST_WHEEL_PWM_PIN1,    0); // Устанавливаем скорость 1-го мотора
        analogWrite(UFO_TEST_WHEEL_PWM_PIN2,    0); // Устанавливаем скорость 2-го мотора
        p=0;
    }

    
    // if (s > 0)
    // {
    //     Serial.println("s>0");
    //     RightFront();
    //     LeftFront();
    //     // Задаём направление для 2-го мотора
    //     digitalWrite(UFO_TEST_WHEEL_W2P1, HIGH);
    //     digitalWrite(UFO_TEST_WHEEL_W2P2, LOW);
    // }
    // else
    // {
    //     Serial.println("s<0");
    //     analogWrite(UFO_TEST_WHEEL_PWM_PIN1,    0); // Устанавливаем скорость 1-го мотора
    //     analogWrite(UFO_TEST_WHEEL_PWM_PIN2,    0); // Устанавливаем скорость 2-го мотора
    //     return;
    // }




    if (r > 180)
    {
        digitalWrite(UFO_TEST_WHEEL_W2P1, LOW);
        digitalWrite(UFO_TEST_WHEEL_W2P2, LOW);
        if (p == 0)
        {
            RightFront();
            LeftBack();
            analogWrite(UFO_TEST_WHEEL_PWM_PIN1, UFO_TEST_WHEEL_ROTATE); 
            analogWrite(UFO_TEST_WHEEL_PWM_PIN2, UFO_TEST_WHEEL_ROTATE); 
            return;
        }
        
    }
    else if (r < -180)
    {
        digitalWrite(UFO_TEST_WHEEL_W1P1, LOW);
        digitalWrite(UFO_TEST_WHEEL_W1P2, LOW);
        if (p == 0)
        {
            LeftFront();
            RightBack();
            analogWrite(UFO_TEST_WHEEL_PWM_PIN1, UFO_TEST_WHEEL_ROTATE); 
            analogWrite(UFO_TEST_WHEEL_PWM_PIN2, UFO_TEST_WHEEL_ROTATE); 
            return;
        }
    }
    else 
    {
        RightFront();
        LeftFront();
    }

    analogWrite(UFO_TEST_WHEEL_PWM_PIN1, 255 *s* p/1000); // Устанавливаем скорость 1-го мотора
    analogWrite(UFO_TEST_WHEEL_PWM_PIN2, 255 *s * p/1000); // Устанавливаем скорость 2-го мотора
}

struct UFO_Motor
{
    UFO_ESC_driver _driver;
    uint16_t _trust = 0;
};
    UFO_Motor LMotor;
    UFO_Motor RMotor;

void TestPinSetup(int){

    LMotor._driver.Setup(0, 13);
    RMotor._driver.Setup(1, 14);
    LMotor._driver.Write(UFO_MOTOR_RES_VAL_MIN);
    RMotor._driver.Write(UFO_MOTOR_RES_VAL_MIN);

}

void TestPinLoop(int p, int r, int){


    // Serial.print(p);
    // Serial.print("\t\t");
    // Serial.print(r);
    // Serial.print("\t\t");
    float w = sqrt( p*p + r*r );
    // Serial.println(p);

    float s = p/w;
    
    // if (s > 0)
    // {
    //     Serial.println("s>0");
    //     RightFront();
    //     LeftFront();
    //     // Задаём направление для 2-го мотора
    //     digitalWrite(UFO_TEST_WHEEL_W2P1, HIGH);
    //     digitalWrite(UFO_TEST_WHEEL_W2P2, LOW);
    // }
    // else
    // {
    //     Serial.println("s<0");
    //     analogWrite(UFO_TEST_WHEEL_PWM_PIN1,    0); // Устанавливаем скорость 1-го мотора
    //     analogWrite(UFO_TEST_WHEEL_PWM_PIN2,    0); // Устанавливаем скорость 2-го мотора
    //     return;
    // }


    int32_t v = 0;

    if (r > 1700)
    {
        if (p < 1300)
        {
            Serial.println("R");

            v = map(1500, UFO_MOTOR_VAL_MIN, UFO_MOTOR_VAL_MAX, UFO_MOTOR_RES_VAL_MIN, UFO_MOTOR_RES_VAL_MAX);
            RMotor._driver.Write(v);

            // analogWrite(UFO_TEST_WHEEL_PWM_PIN1, UFO_TEST_WHEEL_ROTATE); 
            // analogWrite(UFO_TEST_WHEEL_PWM_PIN2, UFO_TEST_WHEEL_ROTATE); 
            return;
        }
        
    }
    else if (r < 1300)
    {
        if (p < 1300)
        {
            Serial.println("L");
            v = map(1500, UFO_MOTOR_VAL_MIN, UFO_MOTOR_VAL_MAX, UFO_MOTOR_RES_VAL_MIN, UFO_MOTOR_RES_VAL_MAX);
            LMotor._driver.Write(v);
            return;
        }
    }

        v = map(p, UFO_MOTOR_VAL_MIN, UFO_MOTOR_VAL_MAX, UFO_MOTOR_RES_VAL_MIN, UFO_MOTOR_RES_VAL_MAX);
        LMotor._driver.Write(v);
        RMotor._driver.Write(v);
        Serial.print("T:");
        Serial.println(p);
        Serial.println(v);
    // analogWrite(UFO_TEST_WHEEL_PWM_PIN1, 255 *s* p/1000); // Устанавливаем скорость 1-го мотора
    // analogWrite(UFO_TEST_WHEEL_PWM_PIN2, 255 *s * p/1000); // Устанавливаем скорость 2-го мотора
}