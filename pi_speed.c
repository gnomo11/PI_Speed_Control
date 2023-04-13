/*			
						CONTROL DIGITAL 
						
					CONTROL PI DE VELOCIDAD 			

				GERARDO NOEL ONOFRE HERNANDEZ 81-S 								*/
			


#include <18F45k50.h>
#fuses NOWDT   
#use delay(internal=48MHz)   

#use timer(TIMER=1, TICK=1ms, BITS=16, ISR) //incremento de cada tick != 1ms, en realidad es de 0.6839ms   

//interrupciones del USB CDC
#define USB_CDC_ISR() ops()
//se define la int|| nombre de la int: "ops"

//aumentar el tamaño del buffer
#define USB_CDC_DELAYED_FLUSH
#define USB_CDC_DATA_LOCAL_SIZE 128

static void ops(void); //Declarar rutina de interrupción del usb-cdc

//librerías
#include <usb_cdc.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>


//----------- MOTOR 1 -----------------------//
#define signal_A PIN_B0 //senal A del encoder -SEÑAL PULSOS DEL ENCODER
#define signal_B PIN_B1 //senal B del encoder - NO ES UTILIZADA
#define gira_derecha PIN_A2
#define gira_izquierda PIN_A3
//--------------------------------------------//

//---------------- VARIABLES DEL PI ------------------//
unsigned int16  lastTime=0, SampleTime=0;
double   Inp=0.0, Setpoint=0.0, out=0.0, out_1=0.0;
double   lastInput=0.0;
double   kp=0.0, ki=0.0;
double   outMin=0.0, outMax=0.0;
//-----------------------------------------------------//
long   pwm=0; // Variable para el PWM (resolución de 0 a 1023).
signed int32 pulsos=0; // Variable contador de pulsos para el encoder.
short  a=0;
double rpm=0.0;

//char data = 0;

int i=0,ini=0,fin=0;
int j=0, q=0;
int16 x1=0, x2=0;
double s1=0.0, s2=0.0,s_p=0.0, den=0.0;
char dat[10];
char degC[10];
char d1[10],d2[10];
short vl=0,el=0,fl=0;


void SetSampleTime(int16 NewSampleTime);//ingresa el tiempo de muestreo
void Compute(void); // CONTROL PID


//------------------------ CONTAR PULSOS MOTOR 1 ------------------------//

#int_ext 
void contar_pulsos(void){

  	pulsos++;
       
//   Configurar para leer tanto en flanco de subida por flanco de bajada
   if(a==0){ext_int_edge(0,L_TO_H);a=1;}
   else {ext_int_edge(0,H_TO_L);a=0;}
	clear_interrupt(INT_EXT); //limpiar bandera de interrupción

}
//----------------------------------------------------------------------//

void main(void){

//------------------------INTERRUPCIONES --------------------------------//
	enable_interrupts(INT_EXT);//HABILITAR INT_EXT_0
	ext_int_edge(0,L_TO_H); //INT_EXT0:INT POR FLANCO DE BAJADA
  	enable_interrupts(GLOBAL); //HABILITAR INT_GLOBALES
//------------------------------------------------------------------//
//---------------------- MOTOR -----------------------------//
  	output_float(signal_A);//señal A del enconder <entrada>
  	output_float(signal_B);//señal B del encoder <entrada>
  	output_drive(gira_derecha); //Salida
  	output_drive(gira_izquierda);//Salida
  	output_high(gira_derecha); //Gira hacia la derecha
  	output_low(gira_izquierda);
//-------------------------------------------------------------------//
//----------------------- TIMERS ---------------------------------//
    setup_timer_1(t1_internal|t1_div_by_8); //configurar TIMER1 
//-----------------------------------------------------------//
//---------------------- PWM -----------------------------//
   setup_timer_2(T2_DIV_BY_16,255,1); // PR2=255 
   setup_ccp1(CCP_PWM); // CCP1 en modo PWM
   set_pwm1_duty(0);
//-----------------------------------------------------------//
   outMax =  1000.0; //Máximo valor del PWM 0-1023
   outMin = -1000.0; //Mínimo valor del PWM

SetSampleTime(44); //44 para que tiempo de muestreo = 30 ms 

//--- Constantes Kp, Ki (a ajustar)-------------------  
   kp = 2.0;   //0.0931; 
   ki = 0.5;   //0.0178;
//---------------------------------------------------
    Setpoint=0.0;
//---------------------- USB_CDC ---------------------------------------
	usb_cdc_init(); //configurar puerto virtual
	usb_init(); //inicializamos el stack USB
	while(!usb_cdc_connected()); //esperar a detectar comunicación
//--------------------------------------------------------------------------
	while(true){
	
		usb_task(); 
		Compute(); //CONTROL PI
		pwm=(long)abs(out); //El valor de accion de control se asigna a la variable pwm
 		set_pwm1_duty(pwm); //"pwm" es el duty_cycle del PWM del microcontrolador

	}
}

/*					CONROL P+I DE VELOCIDAD 			*/
void Compute(void){
 
   double delta = 0.0;

   unsigned int16 now = get_ticks(); //obtiene los ticks contados por TMR1
   unsigned int16 timeChange = (now - lastTime); 

   if (timeChange >= SampleTime) {

		delta = 30.0/1000.0; //tiempo de muestreo 
		Inp = (double)pulsos;
        rpm=(Inp*60.0)/(4200.0*delta); //Calculamos la velocidad
     
		printf(usb_cdc_putc,"\n");
    	printf(usb_cdc_putc,"%3.1f",rpm); //enviar al PC la velocidad
    	printf(usb_cdc_putc,"\n");

         pulsos=0; //reiniciar el conteo de pulsos

  // Calculamos la función de salida del PID.
      out = -kp*(rpm-lastInput)+ ki*(Setpoint-rpm) + out_1;            
     
      if (out > outMax) out = outMax; else if (out < outMin) out = outMin; //limitar la accion de control
     
 // Guardamos el valor de algunas variables para el próximo cálculo
     lastInput = rpm;
     out_1=out;
	 lastTime  = now;   
   }
}


 // 	RUTINA DE INTERRUPCIÓN USB CDC
static void ops(void){

		while(usb_cdc_kbhit()){ //verifica si hay un caracter...


//		Almacenar los datos recibidos [GXX.XF] --> S.P

			while(vl!=1){ //&&i<6
			
				dat[i]=usb_cdc_getc(); //cada caracter se aloja en el vector
				if (dat[i]=='F'){vl=1;}
				i++;
			}

/*------------------------------------------------------------------------------------------------------*/ 
				
					while(el!=1){
						if(dat[j+1]=='.'){el=1;ini=j+2;}
						d1[j]=dat[j+1];
						j++;
					}
					
					if(el==1){
						x1=atol(d1);el=0;
						s1=(double)x1;
						}
					
					while(fl!=1){
						if(dat[ini+q]=='F'){fl=1;fin=ini+q-1;}
						d2[q]=dat[ini+q];
						q++;
					}

					
					for(i=ini;i<=fin;i++){
						degC[i-ini]=dat[i];
					}

					if(fl==1){
						x2=atol(degC);fl=0;
						s2=(double)x2;
						den=(double)q-1.0;
						s2=s2/(pow(10.0,den));					
					}

		//UNA VEZ DECODIFICADOS LOS DATOS RECIBIDOS, SE ASIGNAN A LA VARIABLE CORRESPONDIENTE

				if(dat[0] == 'G'){s_p=s1+s2;Setpoint=s_p;}

				if(dat[0] == 'P'){s_p=s1+s2;kp=s_p;}	

				if(dat[0] == 'I'){s_p=s1+s2;ki=s_p;}

						for(i=ini;i<=fin;i++){degC[i-ini]="";}						
						i=0;j=0;q=0;vl=0;s1=0.0;s2=0.0;	
/*------------------------------------------------------------------------------------------------------*/

		}
	
}

//verifica que el tiempo de muestreo ingresado sea mayor a cero
void SetSampleTime(int16 NewSampleTime){

   if (NewSampleTime > 0) SampleTime = NewSampleTime; 

}

