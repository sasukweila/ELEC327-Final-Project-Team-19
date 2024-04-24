#include <msp430.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

char printdist[] = "Distance to the nearest object is: ";
char centimeter[] = " cm";
char dot[] = ".";
char zero[] = "0";
char newline[] = " \r\n";

int flag = 0;//flag to start running the motor

//temp variables for distance and serial output
volatile int temp[2];
volatile float diff;
volatile unsigned int i=0;
int dst_int;
int dst_flt;
float tmp_flt;
char dst_char[5];
char dst_flt_char[5];

volatile float distance; //distance measured by sensor

void ser_output(char *str);

#define MOTOR1_IN1 BIT0 //IN1 at P1.0
#define MOTOR1_IN2 BIT1 //IN2 at P1.1
#define MOTOR1_EN BIT2 //EN1 at P1.2

//#define MOTOR2_IN3 BIT2 //IN3 at P2.2
//#define MOTOR2_IN4 BIT7 //IN4 at P1.7
//#define MOTOR2_EN BIT3 //EN2 at P1.3

#define PWM_PERIOD 1000

void PWM() {
    //set output direction for the EN pins
    P1DIR |= MOTOR1_EN + MOTOR1_IN1 + MOTOR1_IN2;
//    P2DIR |= MOTOR2_EN + MOTOR2_IN3 + MOTOR2_IN4;
}


void setMotorDirection(int motor, int direction) {
    switch (motor) {
        case 1: //we only need 1 case as both motors are connected to the same OUT pins, but more cases can be added for independent control of wheels
            if (direction == 1) {//turn motor on
                P1OUT &= ~MOTOR1_IN1;
                P1OUT &= ~MOTOR1_IN2;
            } else if (direction == 0) {//turn motor off
                P1OUT |= MOTOR1_IN1;
                P1OUT &= ~MOTOR1_IN2;
            }
            break;
        default:
            break;
    }
}

void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    //set DCO to 1MHz
    BCSCTL1= CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;

    //button config
    P1DIR &= ~BIT3;
    P1OUT &= ~BIT3;
    P1IE |= BIT3;
    P1IES |= BIT3;

    //output config
    P1DIR = BIT6;
    P2SEL = BIT1;
    P1SEL = BIT1|BIT2|BIT6;
    P1SEL2 = BIT1|BIT2;

    //serial output config
    UCA0CTL1 |= UCSWRST+UCSSEL_2;
    UCA0BR0 = 52;
    UCA0BR1 = 0;
    UCA0MCTL = UCBRS_0;
    UCA0CTL1 &= ~UCSWRST;

    //timer config
    TA0CTL = TASSEL_2|MC_1 ;
    TA0CCR0 = 0xFFFF;
    TA0CCR1 = 0x000A;
    TA0CCTL1 = OUTMOD_7;
    TA1CTL = TASSEL_2|MC_2 ;
    TA1CCTL1 = CAP | CCIE | CCIS_0 | CM_3 | SCS ;

    PWM();

    _enable_interrupts();//enable global interrupts

    while(1){
        distance = diff/58;
        dst_int = floor(distance);
        tmp_flt = distance - dst_int;
        ltoa(dst_int, dst_char,10);
        if (tmp_flt < 0.01) {
            dst_flt = floor(tmp_flt * 1000);
            ltoa(dst_flt,dst_flt_char,10);
            ser_output(printdist);
            ser_output(dst_char);
            ser_output(dot);
            ser_output(zero);
            ser_output(zero);
            ser_output(dst_flt_char);
            ser_output(centimeter);
        }
        else if (tmp_flt < 0.1) {
            dst_flt = floor(tmp_flt * 100);
            ltoa(dst_flt,dst_flt_char,10);
            ser_output(printdist);
            ser_output(dst_char);
            ser_output(dot);
            ser_output(zero);
            ser_output(dst_flt_char);
            ser_output(centimeter);
        }
        else {
            dst_flt = floor(tmp_flt * 100);
            ltoa(dst_flt,dst_flt_char,10);
            ser_output(printdist);
            ser_output(dst_char);
            ser_output(dot);
            ser_output(dst_flt_char);
            ser_output(centimeter);
        }
        ser_output(newline);
    }
}
//Timer ISR
#pragma vector = TIMER1_A1_VECTOR
__interrupt void Timer_A(void){
        temp[i] = TA1CCR1;
        i += 1;
        TA1CCTL1 &= ~CCIFG ;
        if (i==2) {
            diff=temp[i-1]-temp[i-2];
            i=0;
        }
        if (abs(distance) <= 15 || !flag) { //if nearest object is less than 15cm away or button flag is disabled
            setMotorDirection(1,0);
            flag = 0;
        } else if(flag){//enables motors once button flag is enabled
            setMotorDirection(1,1);
        }

}
void ser_output(char *str){
    while(*str != 0) {
        while (!(IFG2&UCA0TXIFG));
        UCA0TXBUF = *str++;
        }
}

//Button ISR
#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
    flag ^= 1;
//    P1OUT ^= BIT0;
//    P1IES ^= BIT3;
    P1IFG &= ~BIT3;

}
