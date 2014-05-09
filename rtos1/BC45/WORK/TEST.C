/*
*********************************************************************************************************
*                           Real time operating system - Final project: Traffic lights
*												Vu Nguyen (1005157)
* Project description:
* Case 1: Initially, there is one car and one pedestrian getting involved in the traffic
* The longitudinal lane stands for car lane
* The horizontal lane with red lines stands for pedestrian lane
* Car moving pattern: Down -> Up -> Down -> Up
* Pedestrian moving pattern: Right -> Left -> Right -> Left
* The traffic lights are based on the real traffic lights systems implemented in Finland, with the traffic
* lights for pedestrian lane having green light and red light and the traffic lights for car lane having
* red light, yellow(amber) light, and green light.
* 	Red light means stop for both pedestrians and cars
* 	Yellow light means slowing down for cars in this project. In some countries, however, yellow light 
*	means accelerating the vehicle.
*   Green light means go for both pedestrians and cars.
*	Flashing green means slowing down for pedestrians. 
*********************************************************************************************************
*/

#include "includes.h"

#define TASK_STK_SIZE	512       	/* Size of each task's stacks (# of WORDs) */
#define N_TASKS       	6       	/* Number of identical tasks */
//macros for car lane
#define Y_START		1				/* Beginning of car lane */
#define	Y_END 		21				/* End of car lane */
#define	Y_SENSOR1	5				/* Location of the first sensor before the crossing line Y_INTER1 */
#define	Y_SENSOR2	15				/* Location of the second sensor before the crossing line Y_INTER2 */
#define	Y_INTER1	8				/* The first crossing line of the pedestrian lane */
#define	Y_INTER2	12				/* The second crossing line of the pedestrian lane */
//macros for pedestrian lane
#define X_START		0				/* Beginning of pedestrian lane */
#define X_END		79				/* End of pedestrian lane */
#define	X_SENSOR1	31				/* Location of the first sensor before the crossing line X_INTER1 */
#define X_SENSOR2	61				/* Location of the second sensor before the crossing line X_INTER2 */
#define X_INTER1	34				/* The first crossing line of the car lane */
#define X_INTER2	58				/* The second crossing line of the pedestrian lane */
//macros for speed
#define SPEED_VERY_SLOW		1000
#define SPEED_SLOW			750
#define SPEED_MEDIUM		500
#define SPEED_FAST			250
#define SPEED_VERY_FAST		150
#define SPEED_SUPER_FAST	50

//#define DEBUG		//uncomment this line to see debugging information when testing/running the project

//user-defined data types
typedef enum {false, true} Tboolean;
typedef enum {red, yellow, green, flashingGreen} Tlight;
typedef enum {UP, DOWN, LEFT, RIGHT} Tdirection;
typedef enum {red_green, red_flashingGreen, green_red, yellow_red} TtrafLightState;
typedef struct {
	INT8U x;
	INT8U y;
} Tlocation;

//global variable
INT8U 			*trafficSignals[] = {"CAR_REQUEST", 	/* Traffic signals to be sent by sensors to the controller */
									 "CAR_FINISH", 
									 "PED_REQUEST", 
									 "PED_FINISH"};
Tlight 			pedLights = green;						/* variable for indicating the status of traffic lights of pedestrian lane */
Tlight 			carLights = green;						/* variable for indicating the status of traffic lights of car lane */
Tlocation 		pedPos;									/* variable for tracking the location of pedestrian */
Tlocation 		carPos;									/* variable for tracking the location of car */
Tdirection 		carDirection = DOWN;					/* variable for tracking the direction of car */
Tdirection 		pedDirection = RIGHT;					/* variable for tracking the direction of pedestrian */
Tboolean 		carLastStop = false;					/* variable for keeping the record of the last stop of car */
Tboolean 		pedLastStop = false;					/* variable for keeping the record of the last stop of pedestrian */

OS_EVENT		*CarMbox;								/* mailbox for communication between car lane sensors and the controller */
OS_EVENT 		*PedMbox;								/* mailbox for communication between pedestrian lane sensors and the controller */

OS_STK 			TaskStk[N_TASKS][TASK_STK_SIZE];
OS_STK 			TaskStartStk[TASK_STK_SIZE];

//function prototypes
        void TaskRTC(void *pdata);						/* real time clock task */
		void TaskCar(void *pdata);						/* task for controlling car */
		void TaskPedestrian(void *pdata);				/* task for controlling pedestrian */
        void TaskCarLaneSensors(void *pdata);
        void TaskPedLaneSensors(void *pdata);
		void TaskTrafficLightsController(void *pdata);
		
        void moveCar(Tlocation *pos, INT16U speed);		/* moveCar() and stopCar() are functions associated with TaskCar() */
        void stopCar(Tlocation *pos);
        void movePed(Tlocation *pos, INT16U speed);		/* movePed() and stopPed() are functions associated with TaskPedestrian() */
        void stopPed(Tlocation *pos);
		void displayCarLights(Tlight lightcolor);		/* displayCarLights() and displayPedLights() are functions associated with TaskTrafficLightsController() */
		void displayPedLights(Tlight lightcolor);
		
		void TaskStart(void *pdata); 					/* start-up task */					
static  void TaskStartCreateTasks(void);
static  void TaskStartDispInit(void);					/* tasking for displaying the graphical interface of the traffic lights system*/


/*
 * Application
 */
void  main (void) {

    PC_DispClrScr(DISP_FGND_WHITE + DISP_BGND_BLACK);      /* Clear the screen                         */

    OSInit();                                              /* Initialize uC/OS-II                      */

    PC_DOSSaveReturn();                                    /* Save environment to return to DOS        */
    PC_VectSet(uCOS, OSCtxSw);                             /* Install uC/OS-II's context switch vector */
	PC_ElapsedInit();
	
	CarMbox = OSMboxCreate((void *)0);					   /* Create an empty mailbox for communication between car lane sensors and the controller*/
	PedMbox = OSMboxCreate((void *)0);					   /* Create an empty mailbox for communication between pedestrian lane sensors and the controller*/
	
    OSTaskCreate(TaskStart, (void *)0, &TaskStartStk[TASK_STK_SIZE - 1], 0);
    OSStart();                                             /* Start multitasking                       */
}


/*
*********************************************************************************************************
*                                              STARTUP TASK
*********************************************************************************************************
*/
void  TaskStart (void *pdata)
{
#if OS_CRITICAL_METHOD == 3                                /* Allocate storage for CPU status register */
    OS_CPU_SR  cpu_sr;
#endif
    char       s[100];
    INT16S     key;


    pdata = pdata;                                         /* Prevent compiler warning                 */

    TaskStartDispInit();                                   /* Initialize the display                   */

    OS_ENTER_CRITICAL();								   /* Used to disable interrupts (see chapter 13)*/
    PC_VectSet(0x08, OSTickISR);                           /* Install uC/OS-II's clock tick ISR        */
    PC_SetTickRate(OS_TICKS_PER_SEC);                      /* Reprogram tick rate                      */
    OS_EXIT_CRITICAL();									   /* Re-enable interrupts*/

    OSStatInit();                                          /* Initialize uC/OS-II's statistics         */

    TaskStartCreateTasks();                                /* Create all the application tasks         */

    for (;;) {


        if (PC_GetKey(&key) == TRUE) {                     /* See if key has been pressed              */
            if (key == 0x1B) {                             /* Yes, see if it's the ESCAPE key          */
                PC_DOSReturn();                            /* Return to DOS                            */
            }
        }

        OSCtxSwCtr = 0;                                    /* Clear context switch counter             */
        OSTimeDlyHMSM(0, 0, 1, 0);                         /* Wait one second                          */
    }
}

/*$PAGE*/
/*
*********************************************************************************************************
*                                        INITIALIZE THE DISPLAY
*********************************************************************************************************
*/

static void TaskStartDispInit(void) {
    PC_DispStr( 0,  0, "  RTOS_Final_Project:Traffic lights    Student name:Vu Nguyen  ID:1005157       ", DISP_FGND_WHITE + DISP_BGND_RED + DISP_BLINK);
    PC_DispStr( 0,  1, "                                  |                       |                     ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
    PC_DispStr( 0,  2, "                                  |                       |                     ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
    PC_DispStr( 0,  3, "                                  |                       |                     ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
    PC_DispStr( 0,  4, "                                  |                       |                     ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
    PC_DispStr( 0,  5, "                                  |                       |                     ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
    PC_DispStr( 0,  6, "                                  |                       |                     ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
    PC_DispStr( 0,  7, "                                  |                       |                     ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
    PC_DispStr( 0,  8, "__________________________________|_______________________|_____________________", DISP_FGND_RED + DISP_BGND_LIGHT_GRAY);
    PC_DispStr( 0,  9, "                                  |                       |                     ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
    PC_DispStr( 0, 10, "                                  |                       |                     ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
    PC_DispStr( 0, 11, "                                  |                       |                     ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
    PC_DispStr( 0, 12, "__________________________________|_______________________|_____________________", DISP_FGND_RED + DISP_BGND_LIGHT_GRAY);
    PC_DispStr( 0, 13, "                                  |                       |                     ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
    PC_DispStr( 0, 14, " '$' represents a pedestrian      |                       |                     ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
    PC_DispStr( 0, 15, " 'X' represents a car             |                       |                     ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
    PC_DispStr( 0, 16, " 'R' represents RED light         |                       |                     ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
    PC_DispStr( 0, 17, " 'G' represents GREEN light       |                       |                     ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
    PC_DispStr( 0, 18, " 'Y' represents YELLOW light      |                       |                     ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
    PC_DispStr( 0, 19, "                                  |                       |                     ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
    PC_DispStr( 0, 20, "                                  |                       |                     ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
    PC_DispStr( 0, 21, "                                  |                       |                     ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
    PC_DispStr( 0, 24, "     <-PRESS 'ESC' TO QUIT->                                                    ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY + DISP_BLINK);
}

static void TaskStartCreateTasks(void) {
	/*
	 * Sensors help to report the traffic signals to the controller regularly, so they are given high priority numbers.
	 * Pedestrians are given priority over cars, and therefore, TaskPedLaneSensors() has a higher priority than TaskCarLaneSensors()
	 * The next importance level is the controller, so priority equal to 3
	 * Likewise, a pedestrian is give priority over a car, so TaskPedestrian() (prio = 4) having a higher priority than TaskCar() (prio = 5)
	 * Least important is the real time clock, so this task has the lowest priority of all (prio = 6)
	 */
	OSTaskCreate(TaskRTC, (void *)0, &TaskStk[0][TASK_STK_SIZE - 1], 6); 						//display RTC task
    OSTaskCreate(TaskCar, (void *)0, &TaskStk[1][TASK_STK_SIZE - 1], 5); 						//car task
	OSTaskCreate(TaskPedestrian, (void *)0, &TaskStk[2][TASK_STK_SIZE - 1], 4); 				//pedestrian task
	OSTaskCreate(TaskTrafficLightsController, (void *)0, &TaskStk[3][TASK_STK_SIZE - 1], 3); 	//traffic lights controller task
	OSTaskCreate(TaskPedLaneSensors, (void *)0, &TaskStk[4][TASK_STK_SIZE - 1], 1); 			//pedestrian sensors task
	OSTaskCreate(TaskCarLaneSensors, (void *)0, &TaskStk[5][TASK_STK_SIZE - 1], 2); 			//car sensors task
}

void TaskRTC(void *pdata) {
	INT32U clk;
	char s[24];

	pdata = pdata;
	while(1) {
		PC_GetDateTime(&s[0]);
        PC_DispStr(50, 24, s, DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
        OSTimeDly(1);						//delay here for the kernel to switch to another task
    }
}

void TaskCar(void *pdata) {
	pdata = pdata;							//avoid compiler warning
	carPos.x = (X_INTER1 + X_INTER2) / 2;	//set initial
	carPos.y = Y_START;						//position for car

    while(1) {
		if (carDirection == DOWN) {
			if (carPos.y == Y_SENSOR1) {
				if (carLights == red || carLights == yellow) {
					do {
						carPos.y += 1;
						moveCar(&carPos, SPEED_MEDIUM);
					} while (carPos.y != Y_INTER1-1); 	//light is red or yellow, so move car slowly to Y_INTER1-1 position (just before the crossing line)
					while (carLights != green)
						stopCar(&carPos);				//stop car until light turns green
				} else {
					carPos.y += 1;
					moveCar(&carPos, SPEED_FAST);
				}
			} else if (carPos.y == Y_END) {
				carDirection = UP;
			} else {
				carPos.y += 1;
				moveCar(&carPos, SPEED_FAST);
			}
		} else { //car is moving up
			if (carPos.y == Y_SENSOR2) {
				if (carLights == red || carLights == yellow) {
					do {
						carPos.y -= 1;
						moveCar(&carPos, SPEED_MEDIUM);
					} while (carPos.y != Y_INTER2+1); 	//light is red or yellow, so move car to Y_INTER2+1 position (just before the crossing line)
					while (carLights != green)
						stopCar(&carPos);				//stop car until light turns green
				} else {
					carPos.y -= 1;
					moveCar(&carPos, SPEED_FAST);
				} 
			} else if (carPos.y == Y_START) {
				carDirection = DOWN;
			} else {
				carPos.y -= 1;
				moveCar(&carPos, SPEED_FAST);
			}
		}

		OSTimeDly(1);									//delay here for the kernel to switch to another task
    }

}

void moveCar(Tlocation *pos, INT16U speed) {
	if (carLastStop == true && carDirection == DOWN) { 
		PC_DispChar(pos->x, pos->y-1, ' ', DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY); //col, row, INT8U c, INT8U color
		carLastStop = false;
	}
	
	if (carLastStop == true && carDirection == UP) {
		PC_DispChar(pos->x, pos->y + 1, ' ', DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY); //col, row, INT8U c, INT8U color
		carLastStop = false;
	}
	
	PC_DispChar(pos->x, pos->y, 'X', DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY); 		//col, row, INT8U c, INT8U color
	OSTimeDlyHMSM(0,0,0,speed);
	if (pos->y == Y_INTER1 || pos->y == Y_INTER2)
		PC_DispChar(pos->x, pos->y, '_', DISP_FGND_RED + DISP_BGND_LIGHT_GRAY);
	else
		PC_DispChar(pos->x, pos->y, ' ', DISP_FGND_RED + DISP_BGND_LIGHT_GRAY);
}

void stopCar(Tlocation *pos) {
	PC_DispChar(pos->x, pos->y, 'X', DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY); 		//col, row, INT8U c, INT8U color
	carLastStop = true;
}

void TaskPedestrian(void *pdata) {
	pdata = pdata;								//avoid compiler warning
	pedPos.x = X_START;							//set initial
	pedPos.y = (Y_INTER1 + Y_INTER2) / 2;		//position for car

    while(1) {
		if (pedDirection == RIGHT) {
			if (pedPos.x == X_SENSOR1) {
				if (pedLights == red || pedLights == yellow) {
					do {
						pedPos.x += 1;
						movePed(&pedPos, SPEED_VERY_SLOW);
					} while (pedPos.x != X_INTER1-1); 	//light is red or yellow, so move pedestrian to X_INTER1-1 position(just before the crossing line)
					while(pedLights != green)
						stopPed(&pedPos);				//stop pedestrian until light turns green
				} else {
					pedPos.x += 1;
					movePed(&pedPos, SPEED_SLOW);
				}
			} else if (pedPos.x == X_END) {
				pedDirection = LEFT;
			} else {
				pedPos.x += 1;
				movePed(&pedPos, SPEED_SLOW);
			}
		} else { //pedestrian is moving left
			if (pedPos.x == X_SENSOR2) {
				if (pedLights == red || pedLights == yellow) {
					do {
						pedPos.x -= 1;
						movePed(&pedPos, SPEED_VERY_SLOW);
					} while (pedPos.x != X_INTER2+1); 	//light is red or yellow, so move car to Y_INTER2+1 position(just before the crossing line)
					while (pedLights != green)
						stopPed(&pedPos);				//stop pedestrian until light turns green
				} else {
					pedPos.x -= 1;
					movePed(&pedPos, SPEED_SLOW);
				}
			} else if (pedPos.x == X_START) {
				pedDirection = RIGHT;
			} else {
				pedPos.x -= 1;
				movePed(&pedPos, SPEED_SLOW);
			}
		}

		OSTimeDly(1);									//delay here for the kernel to switch to another task
    }
}

void movePed(Tlocation *pos, INT16U speed) {
	if (pedLastStop == true && pedDirection == RIGHT) {
		PC_DispChar(pos->x-1, pos->y, ' ', DISP_FGND_RED + DISP_BGND_LIGHT_GRAY); //col, row, INT8U c, INT8U color
		pedLastStop = false;
	}
	
	if (pedLastStop == true && pedDirection == LEFT) {
		PC_DispChar(pos->x+1, pos->y, ' ', DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY); //col, row, INT8U c, INT8U color
		pedLastStop = false;
	}
	
	PC_DispChar(pos->x, pos->y, '$', DISP_FGND_RED + DISP_BGND_LIGHT_GRAY); 		//col, row, INT8U c, INT8U color
	OSTimeDlyHMSM(0,0,0,speed);
	if (pos->x == X_INTER1 || pos->x == X_INTER2)
		PC_DispChar(pos->x, pos->y, '|', DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	else
		PC_DispChar(pos->x, pos->y, ' ', DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
}

void stopPed(Tlocation *pos) {
	PC_DispChar(pos->x, pos->y, '$', DISP_FGND_RED + DISP_BGND_LIGHT_GRAY); 		//col, row, INT8U c, INT8U color
	pedLastStop = true;
}

void TaskCarLaneSensors(void *pdata) { 
	INT8U err;
	
	pdata = pdata;
	while(1) {
		if ((carPos.y == Y_SENSOR1 && carDirection == DOWN) || (carPos.y == Y_SENSOR2 && carDirection == UP)) {
			err = OSMboxPost(CarMbox, (void *)trafficSignals[0]);		//send request to cross signal to the controller
			
			#ifdef DEBUG
			switch(err) {
				case OS_NO_ERR:
					PC_DispStr(X_INTER2+1, Y_INTER2+2, "CAR REQUEST", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					OSTimeDlyHMSM(0, 0, 0, 250);
					PC_DispStr(X_INTER2+1, Y_INTER2+2, "           ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					break;
					
				case OS_MBOX_FULL:
					PC_DispStr(X_INTER2+1, Y_INTER2+1, "OS_MBOX_FULL", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					OSTimeDlyHMSM(0, 0, 0, 250);
					PC_DispStr(X_INTER2+1, Y_INTER2+1, "            ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					break;
					
				case OS_ERR_EVENT_TYPE:
					PC_DispStr(X_INTER2+1, Y_INTER2+1, "OS_ERR_EVENT_TYPE", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					OSTimeDlyHMSM(0, 0, 0, 250);
					PC_DispStr(X_INTER2+1, Y_INTER2+1, "            ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					break;
					
				case OS_ERR_PEVENT_NULL:
					PC_DispStr(X_INTER2+1, Y_INTER2+1, "OS_ERR_PEVENT_NULL", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					OSTimeDlyHMSM(0, 0, 0, 250);
					PC_DispStr(X_INTER2+1, Y_INTER2+1, "            ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					break;
					
				default:
					PC_DispStr(X_INTER2+1, Y_INTER2+1, "OS_ERR_POST_NULL_PTR", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					OSTimeDlyHMSM(0, 0, 0, 250);
					PC_DispStr(X_INTER2+1, Y_INTER2+1, "            ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					break;
			}
			#endif
		}
		
		if ((carPos.y == Y_SENSOR2 && carDirection == DOWN) || (carPos.y == Y_SENSOR1 && carDirection == UP)) {
			err = OSMboxPost(CarMbox, (void *)trafficSignals[1]);		//send finish crossing signal to the controller
			
			#ifdef DEBUG
			switch(err) {
				case OS_NO_ERR:
					PC_DispStr(X_INTER2+1, Y_INTER2+4, "CAR FINISH", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					OSTimeDlyHMSM(0, 0, 0, 250);
					PC_DispStr(X_INTER2+1, Y_INTER2+4, "           ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					break;
					
				case OS_MBOX_FULL:
					PC_DispStr(X_INTER2+1, Y_INTER2+3, "OS_MBOX_FULL", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					OSTimeDlyHMSM(0, 0, 0, 250);
					PC_DispStr(X_INTER2+1, Y_INTER2+3, "            ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					break;
					
				case OS_ERR_EVENT_TYPE:
					PC_DispStr(X_INTER2+1, Y_INTER2+3, "OS_ERR_EVENT_TYPE", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					OSTimeDlyHMSM(0, 0, 0, 250);
					PC_DispStr(X_INTER2+1, Y_INTER2+3, "            ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					break;
					
				case OS_ERR_PEVENT_NULL:
					PC_DispStr(X_INTER2+1, Y_INTER2+3, "OS_ERR_PEVENT_NULL", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					OSTimeDlyHMSM(0, 0, 0, 250);
					PC_DispStr(X_INTER2+1, Y_INTER2+3, "            ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					break;
					
				default:
					PC_DispStr(X_INTER2+1, Y_INTER2+3, "OS_ERR_POST_NULL_PTR", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					OSTimeDlyHMSM(0, 0, 0, 250);
					PC_DispStr(X_INTER2+1, Y_INTER2+3, "            ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					break;
			}
			#endif
		}
		OSTimeDly(1);	//delay here for the kernel to switch to another task
	} 
}

void TaskPedLaneSensors(void *pdata) { 
	INT8U err;
	char buf[20];
	pdata = pdata;
	while(1) {
		if ((pedPos.x == X_SENSOR1 && pedDirection == RIGHT) || (pedPos.x == X_SENSOR2 && pedDirection == LEFT)) {
			err = OSMboxPost(PedMbox, (void *)trafficSignals[2]);		//send request to cross signal to the controller
			#ifdef DEBUG
			switch (err) {
				case OS_NO_ERR:
					PC_DispStr(X_START+1, Y_INTER1-2, "PED REQUEST", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					OSTimeDlyHMSM(0, 0, 0, 250);
					PC_DispStr(X_START+1, Y_INTER1-2, "           ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					break;
			
				case OS_MBOX_FULL:
					PC_DispStr(X_START+1, Y_INTER1-1, "OS_MBOX_FULL", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					OSTimeDlyHMSM(0, 0, 0, 250);
					PC_DispStr(X_START+1, Y_INTER1-1, "            ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					break;
					
				case OS_ERR_EVENT_TYPE:
					PC_DispStr(X_START+1, Y_INTER1-1, "OS_ERR_EVENT_TYPE", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					OSTimeDlyHMSM(0, 0, 0, 250);
					PC_DispStr(X_START+1, Y_INTER1-1, "            ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					break;
					
				case OS_ERR_PEVENT_NULL:
					PC_DispStr(X_START+1, Y_INTER1-1, "OS_ERR_PEVENT_NULL", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					OSTimeDlyHMSM(0, 0, 0, 250);
					PC_DispStr(X_START+1, Y_INTER1-1, "            ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					break;
					
				default:
					PC_DispStr(X_START+1, Y_INTER1-1, "OS_ERR_POST_NULL_PTR", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					OSTimeDlyHMSM(0, 0, 0, 250);
					PC_DispStr(X_START+1, Y_INTER1-1, "            ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					break;
			}
			#endif
		}
		
		if ((pedPos.x == X_SENSOR2 && pedDirection == RIGHT) || (pedPos.x == X_SENSOR1 && pedDirection == LEFT)) {
			err = OSMboxPost(PedMbox, (void *)trafficSignals[3]);		//send finish crossing signal to the controller
			#ifdef DEBUG
			switch(err) {
				case OS_NO_ERR:
					PC_DispStr(X_START+1, Y_INTER1-4, "PED FINISH", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					OSTimeDlyHMSM(0, 0, 0, 250);
					PC_DispStr(X_START+1, Y_INTER1-4, "          ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					break;
					
				case OS_MBOX_FULL:
					PC_DispStr(X_START+1, Y_INTER1-3, "OS_MBOX_FULL", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					OSTimeDlyHMSM(0, 0, 0, 250);
					PC_DispStr(X_START+1, Y_INTER1-3, "            ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					break;
					
				case OS_ERR_EVENT_TYPE:
					PC_DispStr(X_START+1, Y_INTER1-3, "OS_ERR_EVENT_TYPE", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					OSTimeDlyHMSM(0, 0, 0, 250);
					PC_DispStr(X_START+1, Y_INTER1-3, "            ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					break;
					
				case OS_ERR_PEVENT_NULL:
					PC_DispStr(X_START+1, Y_INTER1-3, "OS_ERR_PEVENT_NULL", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					OSTimeDlyHMSM(0, 0, 0, 250);
					PC_DispStr(X_START+1, Y_INTER1-3, "            ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					break;
					
				default:
					PC_DispStr(X_START+1, Y_INTER1-3, "OS_ERR_POST_NULL_PTR", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					OSTimeDlyHMSM(0, 0, 0, 250);
					PC_DispStr(X_START+1, Y_INTER1-3, "            ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
					break;
			}
			#endif
		}
		OSTimeDly(1);	
	}
}

void TaskTrafficLightsController(void *pdata) {
	/*
	 * The traffic lights controller is implemented using finite state machine (FSM)
	 * There are 4 states in the FSM: green_red state, yellow_red state, red_green state, and red_flashingGreen state
	 * The controller is simulated based on the real life traffic lights in Finland
	 * However, the small difference is that it controls based on the sensors located at 4 different locations before each of the 
	 * crossing lines.
	 */
	void *carMsg = "";
	void *pedMsg = "";
	TtrafLightState currentState = green_red;
	TtrafLightState nextState = yellow_red;
	
	pdata = pdata;
	while(1) {
		switch(currentState) {
			case green_red:
				#ifdef DEBUG
				PC_DispStr(X_START+1, Y_END-1, "                          ", DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY);
				PC_DispStr(X_START+1, Y_END-1, "in green_red state", DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY);
				#endif
				carLights = green;
				pedLights = red;
				displayCarLights(green);
				displayPedLights(red);
				
				carMsg = OSMboxAccept(CarMbox);
				#ifdef DEBUG
				if (carMsg != NULL) {
					PC_DispStr(X_INTER2+1, Y_START+1, (INT8U *)carMsg, DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY);
					OSTimeDlyHMSM(0, 0, 0, 500);
					PC_DispStr(X_INTER2+1, Y_START+1, "             ", DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY);
				}	
				#endif
				
				pedMsg = OSMboxAccept(PedMbox);
				#ifdef DEBUG
				if (pedMsg != NULL) {
					PC_DispStr(X_START+1, Y_START+1, (INT8U *)pedMsg, DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY);
					OSTimeDlyHMSM(0, 0, 0, 500);
					PC_DispStr(X_START+1, Y_START+1, "           ", DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY);
				}
				#endif
				if (pedMsg != "PED_REQUEST") {
					currentState = currentState;
				} else {
					nextState = yellow_red;
					currentState = nextState;
				}
				break;
		
			case yellow_red:
				#ifdef DEBUG
				PC_DispStr(X_START+1, Y_END-1, "                          ", DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY);
				PC_DispStr(X_START+1, Y_END-1, "in yellow_red state", DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY);
				#endif
				carLights = yellow;
				pedLights = red;
				displayCarLights(yellow);
				displayPedLights(red);
				
				pedMsg = OSMboxAccept(PedMbox);
				#ifdef DEBUG
				if (pedMsg != NULL) {
					PC_DispStr(X_START+1, Y_START+1, (INT8U *)pedMsg, DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY);
					OSTimeDlyHMSM(0, 0, 0, 500);
					PC_DispStr(X_START+1, Y_START+1, "           ", DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY);
				}
				#endif
				
				carMsg = OSMboxAccept(CarMbox);
				#ifdef DEBUG
				if (carMsg != NULL) {
					PC_DispStr(X_INTER2+1, Y_START+1, (INT8U *)carMsg, DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY);
					OSTimeDlyHMSM(0, 0, 0, 500);
					PC_DispStr(X_INTER2+1, Y_START+1, "             ", DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY);
				}	
				#endif
				if (carMsg != "CAR_REQUEST" || carMsg == "CAR_FINISH") {
					nextState = red_green;
					currentState = nextState;
				} else {
					currentState = currentState;
				}
				break;
		
			case red_green:
				#ifdef DEBUG
				PC_DispStr(X_START+1, Y_END-1, "                          ", DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY);
				PC_DispStr(X_START+1, Y_END-1, "in red_green state", DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY);
				#endif
				carLights = red;
				pedLights = green;
				displayCarLights(red);
				displayPedLights(green);
				
				pedMsg = OSMboxAccept(PedMbox);
				#ifdef DEBUG
				if (pedMsg != NULL) {
					PC_DispStr(X_START+1, Y_START+1, (INT8U *)pedMsg, DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY);
					OSTimeDlyHMSM(0, 0, 0, 500);
					PC_DispStr(X_START+1, Y_START+1, "           ", DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY);
				}
				#endif
				
				carMsg = OSMboxAccept(CarMbox);
				#ifdef DEBUG
				if (carMsg != NULL) {
					PC_DispStr(X_INTER2+1, Y_START+1, (INT8U *)carMsg, DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY);
					OSTimeDlyHMSM(0, 0, 0, 500);
					PC_DispStr(X_INTER2+1, Y_START+1, "             ", DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY);
				}	
				#endif
				if (carMsg != "CAR_REQUEST") {
					currentState = currentState;
				} else {
					nextState = red_flashingGreen;
					currentState = nextState;
				}
				break;
		
			case red_flashingGreen:
				#ifdef DEBUG
				PC_DispStr(X_START+1, Y_END-1, "                          ", DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY);
				PC_DispStr(X_START+1, Y_END-1, "in red_flashingGreen state", DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY);
				#endif
				carLights = red;
				pedLights = flashingGreen;
				displayCarLights(red);
				displayPedLights(flashingGreen);
				
				carMsg = OSMboxAccept(CarMbox);
				#ifdef DEBUG
				if (carMsg != NULL) {
					PC_DispStr(X_INTER2+1, Y_START+1, (INT8U *)carMsg, DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY);
					OSTimeDlyHMSM(0, 0, 0, 500);
					PC_DispStr(X_INTER2+1, Y_START+1, "             ", DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY);
				}	
				#endif
				
				pedMsg = OSMboxAccept(PedMbox);
				#ifdef DEBUG
				if (pedMsg != NULL) {
					PC_DispStr(X_START+1, Y_START+1, (INT8U *)pedMsg, DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY);
					OSTimeDlyHMSM(0, 0, 0, 500);
					PC_DispStr(X_START+1, Y_START+1, "           ", DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY);
				}
				#endif
				if (pedMsg == "PED_REQUEST" || pedMsg != "PED_FINISH") {
					currentState = currentState;
				} else {
					nextState = green_red;
					currentState = nextState;
				}
				break;
			
			default:
				PC_DispStr(X_START+1, Y_END-1, "in default state", DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY);
				carLights = red;
				pedLights = green;
				displayCarLights(red);
				displayPedLights(green);
				
				pedMsg = OSMboxAccept(PedMbox);
				#ifdef DEBUG
				if (pedMsg != NULL) {
					PC_DispStr(X_START+1, Y_START+1, (INT8U *)pedMsg, DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY);
					OSTimeDlyHMSM(0, 0, 0, 500);
					PC_DispStr(X_START+1, Y_START+1, "           ", DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY);
				}
				#endif
				
				carMsg = OSMboxAccept(CarMbox);
				#ifdef DEBUG
				if (carMsg != NULL) {
					PC_DispStr(X_INTER2+1, Y_START+1, (INT8U *)carMsg, DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY);
					OSTimeDlyHMSM(0, 0, 0, 500);
					PC_DispStr(X_INTER2+1, Y_START+1, "             ", DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY);
				}	
				#endif
				nextState == red_flashingGreen;
				currentState = nextState;
				break;
		}
		
		OSTimeDly(1);	//delay here for the kernel to switch to another task
	}
}

void displayCarLights(Tlight lightcolor) {
	switch (lightcolor) {
	case red:
		PC_DispChar(X_INTER1+1, Y_INTER1-3, ' ', DISP_FGND_GREEN + DISP_BGND_LIGHT_GRAY); 	//col, row, INT8U c, INT8U color
		PC_DispChar(X_INTER1+1, Y_INTER1-2, ' ', DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY); 	//col, row, INT8U c, INT8U color
		PC_DispChar(X_INTER1+1, Y_INTER1-1, 'R', DISP_FGND_RED + DISP_BGND_LIGHT_GRAY); 	//col, row, INT8U c, INT8U color
		break;
	case yellow:
		PC_DispChar(X_INTER1+1, Y_INTER1-1, ' ', DISP_FGND_RED + DISP_BGND_LIGHT_GRAY); 	//col, row, INT8U c, INT8U color
		PC_DispChar(X_INTER1+1, Y_INTER1-3, ' ', DISP_FGND_GREEN + DISP_BGND_LIGHT_GRAY); 	//col, row, INT8U c, INT8U color
		PC_DispChar(X_INTER1+1, Y_INTER1-2, 'Y', DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY); 	//col, row, INT8U c, INT8U color
		break;
	case green:
		PC_DispChar(X_INTER1+1, Y_INTER1-1, ' ', DISP_FGND_RED + DISP_BGND_LIGHT_GRAY); 	//col, row, INT8U c, INT8U color
		PC_DispChar(X_INTER1+1, Y_INTER1-2, ' ', DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY); 	//col, row, INT8U c, INT8U color
		PC_DispChar(X_INTER1+1, Y_INTER1-3, 'G', DISP_FGND_GREEN + DISP_BGND_LIGHT_GRAY); 	//col, row, INT8U c, INT8U color
		break;
	default:
		PC_DispChar(X_INTER1+1, Y_INTER1-3, ' ', DISP_FGND_GREEN + DISP_BGND_LIGHT_GRAY); 	//col, row, INT8U c, INT8U color
		PC_DispChar(X_INTER1+1, Y_INTER1-2, ' ', DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY); 	//col, row, INT8U c, INT8U color
		PC_DispChar(X_INTER1+1, Y_INTER1-1, 'R', DISP_FGND_RED + DISP_BGND_LIGHT_GRAY); 	//col, row, INT8U c, INT8U color
		break;
	}	
}

void displayPedLights(Tlight lightcolor) {
	INT8U i = 0;
	
	switch (lightcolor) {
	case red:
		PC_DispChar(X_INTER1-1, Y_INTER2-1, ' ', DISP_FGND_GREEN + DISP_BGND_LIGHT_GRAY); 	//col, row, INT8U c, INT8U color
		PC_DispChar(X_INTER1-2, Y_INTER2-1, 'R', DISP_FGND_RED + DISP_BGND_LIGHT_GRAY); 	//col, row, INT8U c, INT8U color
		break;
	case green:
		PC_DispChar(X_INTER1-2, Y_INTER2-1, ' ', DISP_FGND_RED + DISP_BGND_LIGHT_GRAY); 	//col, row, INT8U c, INT8U color
		PC_DispChar(X_INTER1-1, Y_INTER2-1, 'G', DISP_FGND_GREEN + DISP_BGND_LIGHT_GRAY); 	//col, row, INT8U c, INT8U color
		break;
	case flashingGreen:
		PC_DispChar(X_INTER1-2, Y_INTER2-1, ' ', DISP_FGND_RED + DISP_BGND_LIGHT_GRAY); 	//col, row, INT8U c, INT8U color
		for (i = 0 ; i < 4 ; i++) {
			PC_DispChar(X_INTER1-1, Y_INTER2-1, ' ', DISP_FGND_GREEN + DISP_BGND_LIGHT_GRAY); //col, row, INT8U c, INT8U color
			OSTimeDlyHMSM(0,0,0,200);
			PC_DispChar(X_INTER1-1, Y_INTER2-1, 'G', DISP_FGND_GREEN + DISP_BGND_LIGHT_GRAY); //col, row, INT8U c, INT8U color
			OSTimeDlyHMSM(0,0,0,200);
		}
		break;
	default:
		PC_DispChar(X_INTER1-2, Y_INTER2-1, ' ', DISP_FGND_RED + DISP_BGND_LIGHT_GRAY); 	//col, row, INT8U c, INT8U color
		PC_DispChar(X_INTER1-1, Y_INTER2-1, 'G', DISP_FGND_GREEN + DISP_BGND_LIGHT_GRAY); 	//col, row, INT8U c, INT8U color
		break;
	}
}







