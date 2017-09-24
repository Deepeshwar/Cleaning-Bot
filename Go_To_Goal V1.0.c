/*
 * DifferentialDrive.c
 *
 * Created: 08-09-2016 20:32:33
 * Author : chirag
 */
 #define F_CPU 8000000UL

 #include <avr/io.h>
 #include <avr/interrupt.h>
 #include "USART_128.h"
 #include "compass_sensor.h"
 #include "movingArray.h"
 #include "TWI_128.h"
 #include <math.h>
 
 
 #define desireX 0
 #define desireY 0

 // software var
 #define PIDLoop_mainLoop_ratio		1
 #define pi							3.1416
 #define rpmMovArrayLength			10
 #define timeInterval				0.03264			// 1024 * 255 / F_CPU // in sec
 #define leftPWM					OCR3A	
 #define rightPWM					OCR3B

 // hardware var
 #define ticksPerRotation			1000
 #define r							5.0
 #define L							22.2
 #define circumference				31.4				//2 * pi * r
 #define vmax						70
 #define Max						400
 #define d_fw						10
 //PE0 and PE1 RX and TX
 //PD0 and PD1 SCL and SDA
 // PE4 and PE5 interrupt pins
 // PB5 and PB6 PWM pins

 struct position {float x; float y; int phi;};
 struct unicycleState {float v; float w;};
 struct differentialState {float leftRPM; float rightRPM;};

 enum {left, right}; //wheel
 enum {lRPM, rRPM, angularVel}; // movingArray, PID

 volatile long ticks[2] = {0, 0};
 volatile long tick1[2] = {0, 0};
 volatile struct position curBotPosition;
 volatile struct position desiredBotPosition;
 volatile struct differentialState curDiffState;
 volatile struct differentialState curDiffState1;
 volatile struct differentialState desiredDiffState;
 int unitTimeCount = 0;
 volatile uint8_t avoid = 0,Final_ratio=50;
 volatile float phi_ref = 0;
 volatile float kp[3] = {0.26,0.26,0.2}, ki[3] = {0.0, 0.0, 0.0}, kd[3] = {1.68, 1.65, 0.0}, E[3] = {0, 0, 0}, e_old[3] = {0, 0, 0}; 
 volatile uint16_t i,j, dis[5];
 const float Xs[5]={18,12.73,0,-12.73,-18},Ys[5]={0,12.73,18,12.73,0};
 const float Dir_Priority[5]={1,1,1,1,1};
 float Xp[5],Yp[5],X[5],Y[5],Xnet=0,Ynet=0;

 int timekeeper = 0;
 volatile int Obstacle_phi,Obstacle_Avoidance_phi,Follow_wall_c_phi,Follow_wall_cc_phi;
 float ang=0;

 float PID(float error,int x) 
 {
	 float pid = 0;
	 pid = (kp[x]*error) + (ki[x]*E[x]) + (kd[x]*(error - e_old[x]));
	 E[x]+=error;
	 e_old[x] = error;
	 return pid;
 }

 inline void Graph_Plot()
 {
	 USART_Transmitchar(0xAB,0);
	 USART_Transmitchar(0xCD,0);
	 USART_Transmitchar(0x08,0);
	 USART_Transmitchar(0x00,0);
	 
	 USART_Transmitchar((int)curDiffState.leftRPM & 0x00FF,0);
	 USART_Transmitchar((((int)curDiffState.leftRPM & 0xFF00) >> 8),0);
	 USART_Transmitchar((int)curDiffState.rightRPM & 0x00FF,0);
	 USART_Transmitchar((((int)curDiffState.rightRPM & 0xFF00) >> 8),0);
	 USART_Transmitchar((int)curBotPosition.x & 0x00FF,0);
	 USART_Transmitchar((((int)curBotPosition.x & 0xFF00) >> 8),0);
	 USART_Transmitchar((int)curBotPosition.phi & 0x00FF,0);
	 USART_Transmitchar((((int)curBotPosition.phi & 0xFF00) >> 8),0);
 }

 float degreeToRad(float degree) {
	 return degree * pi / 180.0;
 }

float radToDegree(float rad) {
	return rad * 180.0 / pi;
}
float normalizeAngle(float degree) {
	return radToDegree(atan2(sin(degreeToRad(degree)), cos(degreeToRad(degree))));
}

void obstacle_avoid()
{
	Xnet = 0;
	Ynet = 0;
	for(i=0;i<5;i++)
	{
		if (dis[i]==0)
		{
			dis[i]=60;			//for better calculation	
			Xp[i]=Xs[i]+Dir_Priority[i]*(dis[i]*cos((i)*45));
			Yp[i]=Ys[i]+Dir_Priority[i]*(dis[i]*sin((i)*45));
			
			X[i]=Xp[i];
			Y[i]=Yp[i];
			
			Xnet += X[i];
			Ynet += Y[i];
		}
		
	}
	if (Xnet!=0 && Ynet!=0)
	{
		//Xnet+=curBotPosition.x;
		//Ynet+=curBotPosition.y;
		
		Obstacle_phi=radToDegree(atan2(Ynet,Xnet));
		if (Xnet>0)
		{
			Obstacle_Avoidance_phi =  90 + (Obstacle_phi);
			
			
		}
		else
		{
			Obstacle_Avoidance_phi =   (Obstacle_phi) - 90;
		}		
		
	
		Obstacle_Avoidance_phi = normalizeAngle(curBotPosition.phi - 90 + Obstacle_Avoidance_phi);
		//Follow_wall_c_phi	=	normalizeAngle(Obstacle_Avoidance_phi-90);      //obstacle.phi+90
		//Follow_wall_cc_phi=	normalizeAngle(Obstacle_Avoidance_phi+90);		//obstacle.phi+270
		
		
	}
	
	  
	  //if(cos(Obstacle_phi+90-desiredBotPosition.phi)>0)
	 	//{
		// 		right turn;
		// 		follow_wall_using left sensor;
		// 		follow right wall;
		//
	 	//}
	 	//else
	 	//{
		 		//left turn;
		 		//follow wall using left sensor;
		
	 	//}
	
	// 	desiredBotPosition.phi=radToDegree(atan2(Ynet,Xnet));
	// 	phi_ref = getHeading();
	// // 	if (desiredBotPosition.phi > 180)
	// 	{
		// 		desiredBotPosition.phi = desiredBotPosition.phi - 360;
	// 	}*/
	
}

 

 float sigmoid(int z) {
	 return tanh(z/30);
 }

 struct unicycleState getDesiredUnicycleState(struct position curBotPosition, struct position desiredBotPosition) 
 {
	 struct unicycleState desiredState;
	 
	 int errDist = sqrt((desiredBotPosition.x - curBotPosition.x) * (desiredBotPosition.x -curBotPosition.x) + (desiredBotPosition.y - curBotPosition.y)*(desiredBotPosition.y - curBotPosition.y));
	 int desiredPhi = 0;
	 if((curBotPosition.x - desiredBotPosition.x) == 0) {
		 if(desiredBotPosition.y > curBotPosition.y) {
			 desiredPhi = 90;
			 } else {
			 desiredPhi = -90;
		 }
		 } else if((curBotPosition.y - desiredBotPosition.y) == 0) {
		 if(desiredBotPosition.x > curBotPosition.x) {
			 desiredPhi = 0;
			 } else {
			 desiredPhi = 180;
		 }
		 } else {
		 desiredPhi = radToDegree(atan2((desiredBotPosition.y - curBotPosition.y) , (desiredBotPosition.x - curBotPosition.x)));
	 }
	 
	 desiredState.v = vmax * sigmoid(errDist);
	 desiredState.w = PID(normalizeAngle(desiredPhi - curBotPosition.phi), angularVel);
	 
	 return desiredState;
 }

 struct differentialState transformUniToDiff(struct unicycleState uniState)
  {
	 struct differentialState diffState;
	 //using the kinematics equations
	 float vleft = (2*uniState.v -L*uniState.w) / (2 * r);
	 float vright = (2*uniState.v + L*uniState.w)/(2 * r);
	 diffState.rightRPM = vright / circumference * 60;
	 diffState.leftRPM = vleft / circumference * 60;
	 return diffState;
 }


 void calculateDiffState()
  {
	 int x;
	 int sampledTicks[] = {ticks[left], ticks[right]};
	 ticks[0] = 0;
	 ticks[1] = 0;
	 for(x = 0; x <2 ; x++) {
		 float rpm = sampledTicks[x] * 0.91911764;    //	constant = 60 / ticksPerRotation / (timeInterval*2);
		 addElement(rpm, x ,0);
	 }
	 curDiffState.leftRPM = getAverage(lRPM,0);
	 curDiffState.rightRPM = getAverage(rRPM,0);
 }


void calculateDiffState1() 
{
	int x;
	int sampledTick1[] = {tick1[left], tick1[right]};
	tick1[0] = 0;
	tick1[1] = 0;

	for(x = 0; x <2 ; x++) {
	float rpm1 = sampledTick1[x] * 0.91911764;    //	constant = 60 / ticksPerRotation / (timeInterval*2);
	
	addElement(rpm1,x,2);
	
	}
curDiffState1.leftRPM = getAverage(lRPM,2);
curDiffState1.rightRPM = getAverage(rRPM,2);
}

 void calculatePos() 
 {
	 curBotPosition.phi = normalizeAngle(90 + phi_ref-getHeading());
	 float leftDist = curDiffState.leftRPM * timeInterval / 60.0 * circumference;
	 float rightDist = curDiffState.rightRPM * timeInterval / 60.0 * circumference;
	 float dist = (leftDist + rightDist) / 2;
	  
	 curBotPosition.x += dist * cos(degreeToRad(curBotPosition.phi));
	 curBotPosition.y += dist * sin(degreeToRad(curBotPosition.phi));
 }


void changeWheelOutputs(struct differentialState curState, struct differentialState desiredState) {
	float leftPID = leftPWM;
	float rightPID = rightPWM;
	
	if(desiredState.leftRPM<0)
	{
		desiredState.leftRPM=(-1)*desiredState.leftRPM;
		PORTE|=(1<<PINE2);
	}
	else
	{
		PORTE&=~(1<<PINE2);
	}
	
	
	if(desiredState.rightRPM<0)
	{
		desiredState.rightRPM=(-1)*desiredState.rightRPM;
		PORTE|=(1<<PINE5);
	}
	else
	{
		PORTE&=~(1<<PINE5);
	}
	
	leftPID += PID(desiredState.leftRPM - curState.leftRPM, left);
	rightPID += PID(desiredState.rightRPM - curState.rightRPM, right);
	

	if(leftPID > Max) {
		leftPID = Max;
	} else if(leftPID < 0){
		leftPID = 0;
	}
	if(rightPID > Max) {
		rightPID = Max;
	} else if(rightPID < 0){
		rightPID = 0;
	}
	
	leftPWM = lround(leftPID);
	rightPWM = lround(rightPID);
	
}

 int main() {
	 
	 _delay_ms(100); // time to let compass sensor load
	 
	 DDRE |= (1<<PINE2) | (1<<PINE3) | (1<<PINE4) | (1<<PINE5);
	 //DDRD |= 0XFF;
	 
	 //interrupt , any logical change
	 EICRA |= (1<<ISC20) | (1<<ISC21)| (1<<ISC30) | (1<<ISC31);
	 EIMSK |= (1<<INT2) | (1<<INT3);
		 
	 //timers 10 bit
	 TCCR0 |= (1<<CS02) | (1<<CS01) | (1<<CS00);
	 TIMSK |= (1<<TOIE0);
	 
	 //PWM_timer , Fast_PWM_mode, Top = 0x03FF(in Hex) or 1023(in Decimal)
	 TCCR3B |= (1<<CS30) | (1<<WGM32);
	 TCCR3A |= (1<<COM3A1) | (1<<COM3B1) | (1 << WGM31) | (1<< WGM30);
	 
	init_HMC5883L();
//	unsigned int i;
// 	TWI_Init();
// 	TWI_WriteRegister(0x42,0x74,0b01100010);
// 	i = TWI_ReadRegisterNACK(0x42,0x01)<<8;
// 	i |= TWI_ReadRegisterNACK(0x42,0x04);

	 USART_Init(51,0);
	 //USART_Init(51,1);
	 USART_InterruptEnable(0);
	 
	 sei();
    
	 phi_ref=getHeading();
	
	 init_movingArray(rpmMovArrayLength, lRPM);
	 init_movingArray(rpmMovArrayLength, rRPM);
	 
	 //taking initial point as origin
	 curBotPosition.x = 0;
	 curBotPosition.y = 0;

	
// 	desiredDiffState.leftRPM=30;
// 	desiredDiffState.rightRPM=30;
 	 
	 desiredBotPosition.x = desireX;
	 desiredBotPosition.y = desireY;
		 
	 while (1) 
	 {
		
	 }
	 
 }

ISR(TIMER0_OVF_vect) {
	 
	/* avoid++;
	 
	 if(avoid==20)
	 {
		 desiredBotPosition.x = desireX;
		 desiredBotPosition.y = desireY;
		 
		 obstacle_avoid();
		 //_delay_ms(10);
		 avoid=0;
		 
	 }
	 */
	unitTimeCount++;
	
	if(timekeeper == 2)
	{
		calculateDiffState1();
		calculateDiffState();
		timekeeper = 0;
	}	timekeeper++;
	
	calculatePos();
	
	if(unitTimeCount == PIDLoop_mainLoop_ratio)
	{
		desiredDiffState = transformUniToDiff(getDesiredUnicycleState(curBotPosition, desiredBotPosition));
		unitTimeCount = 0;
	}
	
	changeWheelOutputs(curDiffState1, desiredDiffState);
	/*
	USART_Transmitchar('l',0);
	USART_TransmitNumber(curDiffState.leftRPM,0);
	USART_Transmitchar(0x0A,0);
	USART_Transmitchar('r',0);
	USART_TransmitNumber(curDiffState.rightRPM,0);
	USART_Transmitchar(0x0A,0);*/
	USART_Transmitchar('x',0);
	USART_TransmitNumber(curBotPosition.x,0);
	USART_Transmitchar('\t',0);
	USART_Transmitchar('y',0);
	USART_TransmitNumber(curBotPosition.y,0);
	USART_Transmitchar('\t',0);
	/*USART_Transmitchar('L',0);
	USART_TransmitNumber(tick1[left],0);
	USART_Transmitchar(0x0A,0);
	USART_Transmitchar('R',0);
	USART_TransmitNumber(tick1[right],0);
	USART_Transmitchar(0x0A,0);*/
	USART_Transmitchar('p',0);
	USART_TransmitNumber(curBotPosition.phi,0);
	/*USART_Transmitchar(0x0A,0);
	USART_Transmitchar('O',0);
	USART_TransmitNumber(Obstacle_Avoidance_phi,0);
	USART_Transmitchar(0x0A,0);
	USART_Transmitchar('o',0);
	USART_TransmitNumber(Obstacle_phi,0);
	USART_Transmitchar(0x0D,0);
// 	USART_Transmitchar('d',0);
// 	USART_TransmitNumber(desiredBotPosition.x,0);
// 	USART_Transmitchar(0x0A,0);
// 	USART_Transmitchar('D',0);
// 	USART_TransmitNumber(desiredBotPosition.y,0);*/
	USART_Transmitchar(0x0D,0);
	//Graph_Plot();*/
}
 ISR(INT2_vect) {
	 if(bit_is_clear(PIND,4))
	 ticks[left]++;
	 else if(bit_is_set(PIND,4))
	 ticks[left]--;
	 
	 tick1[left]++;
	 
 }


 ISR(INT3_vect) {
	 if(bit_is_clear(PIND,7))
	 ticks[right]++; 
	 else if(bit_is_set(PIND,7))
	 ticks[right]--;
	 
	 tick1[right]++;
 }
 
 /*ISR(USART0_RX_vect)
 {
	 char distance;
	 
	 distance=USART_Receive(0);
	 
	 if(distance == 'A')
	 {
		 j=0;
		 dis[0]=0;
	 }
	 else if (distance == 'B')
	 {
		 j=1;
		 dis[1]=0;
	 }
	 else if (distance == 'C')
	 {
		 j=2;
		 dis[2]=0;
	 }
	 else if (distance == 'D')
	 {
		 j=3;
		 dis[3]=0;
	 }
	 else if (distance == 'E')
	 {
		 j=4;
		 dis[4]=0;
	 }
	 else
	 {
		 
		 dis[j]=dis[j]*10 + (distance-'0');
	 }
	 
 }*/
 USART_Receive(0)
{
	char m;
	
}