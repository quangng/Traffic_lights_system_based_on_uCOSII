/*
*********************************************************************************************************
*                           Real time operating system - Final project: Traffic lights
*												Vu Nguyen (1005157)
* Project description:
* Case 2: Initially, there are three cars and one pedestrian getting involved in the traffic
* The three cars should be able to communicate with each other in order to come across the road
* The longitudinal lane stands for car lane
* The horizontal lane with red lines stands for pedestrian lane
* Car moving pattern: Down -> Right -> Up -> Left -> Down
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
#define N_TASKS       	8       	/* Number of identical tasks */
//macros for car lane
#define Y_START		1				/* Beginning of car lane */
#define	Y_END 		23				/* End of car lane */
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
									 
INT8U			*carSignals[] = {"STOP", "GO", "ACK"};	/* signals for communication between cars*/									 								 
Tlight 			pedLights = green;						/* variable for indicating the status of traffic lights of pedestrian lane */
Tlight 			carLights = green;						/* variable for indicating the status of traffic lights of car lane */
Tlocation 		pedPos;									/* variable for tracking the location of pedestrian */
Tlocation 		car1Pos;								/* variable for tracking the location of car 1*/
Tlocation 		car2Pos;								/* variable for tracking the location of car 2*/
Tlocation 		car3Pos;								/* variable for tracking the location of car 3*/
Tdirection 		car1Direction = DOWN;					/* variable for tracking the direction of car 1*/
Tdirection 		car2Direction = DOWN;					/* variable for tracking the direction of car 2*/
Tdirection 		car3Direction = DOWN;					/* variable for tracking the direction of car 3*/
Tdirection 		pedDirection = RIGHT;					/* variable for tracking the direction of pedestrian */
Tboolean 		car1LastStop = false;					/* variable for keeping the record of the last stop of car 1*/
Tboolean 		car2LastStop = false;					/* variable for keeping the record of the last stop of car 2*/
Tboolean 		car3LastStop = false;					/* variable for keeping the record of the last stop of car 3*/
Tboolean 		pedLastStop = false;					/* variable for keeping the record of the last stop of pedestrian */

OS_EVENT		*CarMbox;								/* mailbox for communication between car lane sensors and the controller */
OS_EVENT 		*PedMbox;								/* mailbox for communication between pedestrian lane sensors and the controller */
OS_EVENT		*Car1ToCar2Mbox;						/* mailbox for communication from car1 to car2 */
OS_EVENT		*Car2ToCar3Mbox;						/* mailbox for communication from car2 to car3 */

OS_STK 			TaskStk[N_TASKS][TASK_STK_SIZE];
OS_STK 			TaskStartStk[TASK_STK_SIZE];

//function prototypes
        void TaskRTC(void *pdata);						/* real time clock task */
		void TaskCar1(void *pdata);						/* task for controlling car 1*/
		void TaskCar2(void *pdata);						/* task for controlling car 2*/
		void TaskCar3(void *pdata);						/* task for controlling car 3*/
		void TaskPedestrian(void *pdata);				/* task for controlling pedestrian */
        void TaskCarLaneSensors(void *pdata);
        void TaskPedLaneSensors(void *pdata);
		void TaskTrafficLightsController(void *pdata);
		
        void moveCar1(Tlocation *pos, INT16U speed);	/* moveCar() and stopCar() are functions associated with TaskCar() */
        void moveCar2(Tlocation *pos, INT16U speed);
        void moveCar3(Tlocation *pos, INT16U speed);
        void stopCar1(Tlocation *pos);
        void stopCar2(Tlocation *pos);
        void stopCar3(Tlocation *pos);
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

    PC_DispClrScr(DISP_FGND_WHITE + DISP_BGND_BLACK);      	/* Clear the screen                         */

    OSInit();                                              	/* Initialize uC/OS-II                      */

    PC_DOSSaveReturn();                                    	/* Save environment to return to DOS        */
    PC_VectSet(uCOS, OSCtxSw);                             	/* Install uC/OS-II's context switch vector */
	PC_ElapsedInit();
	
	CarMbox = OSMboxCreate((void *)0);					   	/* Create an empty mailbox for communication between car lane sensors and the controller*/
	PedMbox = OSMboxCreate((void *)0);					   	/* Create an empty mailbox for communication between pedestrian lane sensors and the controller*/
	Car1ToCar2Mbox = OSMboxCreate((void *)0);				/* Create an empty mailbox for communication from car1 to car2*/
	Car2ToCar3Mbox = OSMboxCreate((void *)0);				/* Create an empty mailbox for communication from car2 to car3*/

    OSTaskCreate(TaskStart, (void *)0, &TaskStartStk[TASK_STK_SIZE - 1], 0);
    OSStart();                                             	/* Start multitasking                       */
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
    PC_DispStr( 0, 15, " 'X' represents car 1             |                       |                     ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
    PC_DispStr( 0, 16, " 'Y' represents car 2             |                       |                     ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
    PC_DispStr( 0, 17, " 'Z' represents car 3             |                       |                     ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
    PC_DispStr( 0, 18, " 'R' represents red light         |                       |                     ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
    PC_DispStr( 0, 19, " 'Y' represents yellow light      |                       |                     ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
    PC_DispStr( 0, 20, " 'G' represents green light       |                       |                     ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
    PC_DispStr( 0, 21, "                                  |                       |                     ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
    PC_DispStr( 0, 22, "                                  |                       |                     ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
    PC_DispStr( 0, 23, "                                  |                       |                     ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
    PC_DispStr( 0, 24, "     <-PRESS 'ESC' TO QUIT->                                                    ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY + DISP_BLINK);
}

static void TaskStartCreateTasks(void) {
	/*
	 * Sensors help to report the traffic signals to the controller regularly, so they are given high priority numbers.
	 * Pedestrians are given priority over cars, and therefore, TaskPedLaneSensors() has a higher priority than TaskCarLaneSensors()
	 * The next importance level is the controller, so priority equal to 3
	 * Likewise, a pedestrian is give priority over a cars, so TaskPedestrian() (prio = 4) having a higher priority than TaskCar1() (prio = 5) and
	 * TaskCar2() (prio = 6) and TaskCar3() (prio = 7)
	 * Least important is the real time clock, so this task has the lowest priority of all (prio = 8)
	 */
	OSTaskCreate(TaskPedLaneSensors, (void *)0, &TaskStk[0][TASK_STK_SIZE - 1], 1); 			//pedestrian sensors task
	OSTaskCreate(TaskCarLaneSensors, (void *)0, &TaskStk[1][TASK_STK_SIZE - 1], 2); 			//car sensors task
	OSTaskCreate(TaskTrafficLightsController, (void *)0, &TaskStk[2][TASK_STK_SIZE - 1], 3); 	//traffic lights controller task
	OSTaskCreate(TaskPedestrian, (void *)0, &TaskStk[3][TASK_STK_SIZE - 1], 4); 				//pedestrian task
    OSTaskCreate(TaskCar1, (void *)0, &TaskStk[4][TASK_STK_SIZE - 1], 5); 						//car 1 task
	OSTaskCreate(TaskCar2, (void *)0, &TaskStk[5][TASK_STK_SIZE - 1], 6); 						//car 2 task
	OSTaskCreate(TaskCar3, (void *)0, &TaskStk[6][TASK_STK_SIZE - 1], 7); 						//car 3 task
	OSTaskCreate(TaskRTC, (void *)0, &TaskStk[7][TASK_STK_SIZE - 1], 8); 						//display RTC task
}

void TaskRTC(void *pdata) {
	INT32U clk;
	char s[24];

	pdata = pdata;
	while(1) {
		PC_GetDateTime(&s[0]);
        PC_DispStr(X_INTER1+16, Y_END+1, s, DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY + DISP_BLINK);
        OSTimeDly(1);						//delay here for the kernel to switch to another task
    }
}

void TaskCar1(void *pdata) {
	pdata = pdata;							//avoid compiler warning
	car1Pos.x = X_INTER1 + 3;				//set initial
	car1Pos.y = Y_START + 4;				//position for car
	
    while(1) {
		switch (car1Direction) {
			case DOWN:
				if (car1Pos.y == Y_SENSOR1) {
					if (carLights == red || carLights == yellow) {
						do {
							car1Pos.y += 1;
							moveCar1(&car1Pos, SPEED_MEDIUM);
							OSMboxPost(Car1ToCar2Mbox, (void *)carSignals[1]);
						} while (car1Pos.y != Y_INTER1-1); 	
						while (carLights != green) {
							stopCar1(&car1Pos);
							OSMboxPost(Car1ToCar2Mbox, (void *)carSignals[0]);											
						} 
					} else {
						car1Pos.y += 1;
						moveCar1(&car1Pos, SPEED_FAST);
						OSMboxPost(Car1ToCar2Mbox, (void *)carSignals[1]);
					}
				} else if (car1Pos.y == Y_END) {
					car1Direction = RIGHT;
				} else {
					car1Pos.y += 1;
					moveCar1(&car1Pos, SPEED_FAST);				
					OSMboxPost(Car1ToCar2Mbox, (void *)carSignals[1]);				
				}
				break;
				
			case RIGHT:
				if (car1Pos.x == X_INTER2-3)
					car1Direction = UP;
				else {
					car1Pos.x += 1;
					moveCar1(&car1Pos, SPEED_FAST);
					OSMboxPost(Car1ToCar2Mbox, (void *)carSignals[1]);
				}
				break;
				
			case UP:
				if (car1Pos.y == Y_SENSOR2) {
					if (carLights == red || carLights == yellow) {
						do {
							car1Pos.y -= 1;
							moveCar1(&car1Pos, SPEED_MEDIUM);
							OSMboxPost(Car1ToCar2Mbox, (void *)carSignals[1]);
						} while (car1Pos.y != Y_INTER2+1); 	
						while (carLights != green) { 
							stopCar1(&car1Pos);		
							OSMboxPost(Car1ToCar2Mbox, (void *)carSignals[0]);
						}		
					} else {
						car1Pos.y -= 1;
						moveCar1(&car1Pos, SPEED_FAST);
						OSMboxPost(Car1ToCar2Mbox, (void *)carSignals[1]);
					} 
				} else if (car1Pos.y == Y_START) {
					car1Direction = LEFT;
				} else {
					car1Pos.y -= 1;
					moveCar1(&car1Pos, SPEED_FAST);	
					OSMboxPost(Car1ToCar2Mbox, (void *)carSignals[1]);		
				}
				break;
				
			case LEFT:
				if (car1Pos.x == X_INTER1+3)
					car1Direction = DOWN;
				else {
					car1Pos.x -= 1;
					moveCar1(&car1Pos, SPEED_FAST);
					OSMboxPost(Car1ToCar2Mbox, (void *)carSignals[1]);			
				}
				break;
		} //end switch case
		OSTimeDly(1);								
	} //end while(1)
} //end task
		
void TaskCar2(void *pdata) {
	INT8U err;
	char *rxMsg;
	
	pdata = pdata;							//avoid compiler warning
	car2Pos.x = X_INTER1 + 3;				//set initial
	car2Pos.y = Y_START + 2;				//position for car
    while(1) {
		if (car2Direction == DOWN) {
			if (car2Pos.y == Y_END) {
				car2Direction = RIGHT;
			} else {
				rxMsg = (char *)OSMboxAccept(Car1ToCar2Mbox);
				if (rxMsg == carSignals[0]) {
					stopCar2(&car2Pos);
					OSMboxPost(Car2ToCar3Mbox, (void *)carSignals[0]);
				} else if (rxMsg == carSignals[1]) {
					car2Pos.y += 1;
					moveCar2(&car2Pos, SPEED_FAST);	
					OSMboxPost(Car2ToCar3Mbox, (void *)carSignals[1]);
				} else {	
					stopCar2(&car2Pos);
					OSMboxPost(Car2ToCar3Mbox, (void *)carSignals[0]);
				}
			}
		} else if (car2Direction == RIGHT) {
			if (car2Pos.x == X_INTER2-3)
				car2Direction = UP;
			else {
				rxMsg = (char *)OSMboxAccept(Car1ToCar2Mbox);
				if (rxMsg == carSignals[0]) {
					stopCar2(&car2Pos);
					OSMboxPost(Car2ToCar3Mbox, (void *)carSignals[0]);	
				} else if (rxMsg == carSignals[1]) {
					car2Pos.x += 1;
					moveCar2(&car2Pos, SPEED_FAST);
					OSMboxPost(Car2ToCar3Mbox, (void *)carSignals[1]);	
				} else {	
					stopCar2(&car2Pos);
					OSMboxPost(Car2ToCar3Mbox, (void *)carSignals[0]);
				}				
			}
		} else if (car2Direction == UP) { 	
			if (car2Pos.y == Y_START) {
				car2Direction = LEFT;
			} else {	
				rxMsg = (char *)OSMboxAccept(Car1ToCar2Mbox);
				if (rxMsg == carSignals[0]) {
					stopCar2(&car2Pos);
					OSMboxPost(Car2ToCar3Mbox, (void *)carSignals[0]);
				} else if (rxMsg == carSignals[1]) {
					car2Pos.y -= 1;
					moveCar2(&car2Pos, SPEED_FAST);
					OSMboxPost(Car2ToCar3Mbox, (void *)carSignals[1]);	
				} else {
					stopCar2(&car2Pos);
					OSMboxPost(Car2ToCar3Mbox, (void *)carSignals[0]);
				}
			}
		} else { 
			if (car2Pos.x == X_INTER1+3)
				car2Direction = DOWN;
			else {
				rxMsg = (char *)OSMboxAccept(Car1ToCar2Mbox);
				if (rxMsg == carSignals[0]) {
					stopCar2(&car2Pos);
					OSMboxPost(Car2ToCar3Mbox, (void *)carSignals[0]);
				} else if (rxMsg == carSignals[1]) {
					car2Pos.x -= 1;
					moveCar2(&car2Pos, SPEED_FAST);
					OSMboxPost(Car2ToCar3Mbox, (void *)carSignals[1]);	
				} else {	
					stopCar2(&car2Pos);
					OSMboxPost(Car2ToCar3Mbox, (void *)carSignals[0]);
				}
			}
		}

		OSTimeDly(1);									
    } //end while(1)
}

void TaskCar3(void *pdata) {
	INT8U *rxMsg;
	
	pdata = pdata;							//avoid compiler warning
	car3Pos.x = X_INTER1 + 3;				//set initial
	car3Pos.y = Y_START;					//position for car
    while(1) {
		if (car3Direction == DOWN) {
			if (car3Pos.y == Y_END) {
				car3Direction = RIGHT;
			} else {
				rxMsg = (INT8U *)OSMboxAccept(Car2ToCar3Mbox);
				if (rxMsg == carSignals[0]) {
					stopCar3(&car3Pos);
				} else if (rxMsg == carSignals[1]) {
					car3Pos.y += 1;
					moveCar3(&car3Pos, SPEED_FAST);	
				} else {
					stopCar3(&car3Pos);
				}
			}
		} else if (car3Direction == RIGHT) {
			if (car3Pos.x == X_INTER2-3)
				car3Direction = UP;
			else {
				rxMsg = (INT8U *)OSMboxAccept(Car2ToCar3Mbox);
				if (rxMsg == carSignals[0]) {
					stopCar3(&car3Pos);
				} else if (rxMsg == carSignals[1]) {
					car3Pos.x += 1;
					moveCar3(&car3Pos, SPEED_FAST);
				} else {
					stopCar3(&car3Pos);
				}
			}
		} else if (car3Direction == UP) { 	
			if (car3Pos.y == Y_START) {
				car3Direction = LEFT;
			} else {
				rxMsg = (INT8U *)OSMboxAccept(Car2ToCar3Mbox);
				if (rxMsg == carSignals[0]) {
					stopCar3(&car3Pos);
				} else if (rxMsg == carSignals[1]) {
					car3Pos.y -= 1;
					moveCar3(&car3Pos, SPEED_FAST);
				} else {
					stopCar3(&car3Pos);
				}
			}
		} else {
			if (car3Pos.x == X_INTER1+3)
				car3Direction = DOWN;
			else {
				rxMsg = (INT8U *)OSMboxAccept(Car2ToCar3Mbox);
				if (rxMsg == carSignals[0]) {
					stopCar3(&car3Pos);
				} else if (rxMsg == carSignals[1]) {
					car3Pos.x -= 1;
					moveCar3(&car3Pos, SPEED_FAST);
				} else {
					stopCar3(&car3Pos);
				}
			}
		}

		OSTimeDly(1);									//delay here for the kernel to switch to another task
    } //end while(1)
}

void moveCar1(Tlocation *pos, INT16U speed) {
	if (car1LastStop == true && car1Direction == DOWN) { 
		PC_DispChar(pos->x, pos->y-1, ' ', DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY); //col, row, INT8U c, INT8U color
		car1LastStop = false;
	}
	
	if (car1LastStop == true && car1Direction == UP) {
		PC_DispChar(pos->x, pos->y + 1, ' ', DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY); //col, row, INT8U c, INT8U color
		car1LastStop = false;
	}
	
	PC_DispChar(pos->x, pos->y, 'X', DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY); 		//col, row, INT8U c, INT8U color
	OSTimeDlyHMSM(0,0,0,speed);
	if (pos->y == Y_INTER1 || pos->y == Y_INTER2)
		PC_DispChar(pos->x, pos->y, '_', DISP_FGND_RED + DISP_BGND_LIGHT_GRAY);
	else
		PC_DispChar(pos->x, pos->y, ' ', DISP_FGND_RED + DISP_BGND_LIGHT_GRAY);
}

void stopCar1(Tlocation *pos) {
	PC_DispChar(pos->x, pos->y, 'X', DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY); 		//col, row, INT8U c, INT8U color
	car1LastStop = true;
}

void moveCar2(Tlocation *pos, INT16U speed) {
	if (car2LastStop == true && car2Direction == DOWN) { 
		PC_DispChar(pos->x, pos->y-1, ' ', DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY); //col, row, INT8U c, INT8U color
		car2LastStop = false;
	}
	
	if (car2LastStop == true && car2Direction == UP) {
		PC_DispChar(pos->x, pos->y + 1, ' ', DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY); //col, row, INT8U c, INT8U color
		car2LastStop = false;
	}
	
	PC_DispChar(pos->x, pos->y, 'Y', DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY); 		//col, row, INT8U c, INT8U color
	OSTimeDlyHMSM(0,0,0,speed);
	if (pos->y == Y_INTER1 || pos->y == Y_INTER2)
		PC_DispChar(pos->x, pos->y, '_', DISP_FGND_RED + DISP_BGND_LIGHT_GRAY);
	else
		PC_DispChar(pos->x, pos->y, ' ', DISP_FGND_RED + DISP_BGND_LIGHT_GRAY);
}

void stopCar2(Tlocation *pos) {
	PC_DispChar(pos->x, pos->y, 'Y', DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY); 		//col, row, INT8U c, INT8U color
	car2LastStop = true;
}

void moveCar3(Tlocation *pos, INT16U speed) {
	if (car3LastStop == true && car3Direction == DOWN) { 
		PC_DispChar(pos->x, pos->y-1, ' ', DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY); //col, row, INT8U c, INT8U color
		car3LastStop = false;
	}
	
	if (car3LastStop == true && car3Direction == UP) {
		PC_DispChar(pos->x, pos->y + 1, ' ', DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY); //col, row, INT8U c, INT8U color
		car3LastStop = false;
	}
	
	PC_DispChar(pos->x, pos->y, 'Z', DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY); 		//col, row, INT8U c, INT8U color
	OSTimeDlyHMSM(0,0,0,speed);
	if (pos->y == Y_INTER1 || pos->y == Y_INTER2)
		PC_DispChar(pos->x, pos->y, '_', DISP_FGND_RED + DISP_BGND_LIGHT_GRAY);
	else
		PC_DispChar(pos->x, pos->y, ' ', DISP_FGND_RED + DISP_BGND_LIGHT_GRAY);
}

void stopCar3(Tlocation *pos) {
	PC_DispChar(pos->x, pos->y, 'Z', DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY); 		//col, row, INT8U c, INT8U color
	car3LastStop = true;
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
	pdata = pdata;
	
	while(1) {
		if ((car1Pos.y == Y_SENSOR1 && car1Direction == DOWN) || (car1Pos.y == Y_SENSOR2 && car1Direction == UP)) 
			OSMboxPost(CarMbox, (void *)trafficSignals[0]);		//send request to cross signal to the controller
		
		if ((car3Pos.y == Y_SENSOR2 && car3Direction == DOWN) || (car3Pos.y == Y_SENSOR1 && car3Direction == UP)) 	
			OSMboxPost(CarMbox, (void *)trafficSignals[1]);		//send finish crossing signal to the controller
		
		OSTimeDly(1);	//delay here for the kernel to switch to another task
	} 
}

void TaskPedLaneSensors(void *pdata) { 
	pdata = pdata;
	
	while(1) {
		if ((pedPos.x == X_SENSOR1 && pedDirection == RIGHT) || (pedPos.x == X_SENSOR2 && pedDirection == LEFT)) 
			OSMboxPost(PedMbox, (void *)trafficSignals[2]);		//send request to cross signal to the controller
		
		if ((pedPos.x == X_SENSOR2 && pedDirection == RIGHT) || (pedPos.x == X_SENSOR1 && pedDirection == LEFT)) 
			OSMboxPost(PedMbox, (void *)trafficSignals[3]);		//send finish crossing signal to the controller
		
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
				carLights = green;
				pedLights = red;
				displayCarLights(green);
				displayPedLights(red);
				
				carMsg = OSMboxAccept(CarMbox);
				pedMsg = OSMboxAccept(PedMbox);
				if (pedMsg != "PED_REQUEST") {
					currentState = currentState;
				} else {
					nextState = yellow_red;
					currentState = nextState;
				}
				break;
		
			case yellow_red:
				carLights = yellow;
				pedLights = red;
				displayCarLights(yellow);
				displayPedLights(red);
				
				pedMsg = OSMboxAccept(PedMbox);
				carMsg = OSMboxAccept(CarMbox);
				if (carMsg != "CAR_REQUEST" || carMsg == "CAR_FINISH") {
					nextState = red_green;
					currentState = nextState;
				} else {
					currentState = currentState;
				}
				break;
		
			case red_green:
				carLights = red;
				pedLights = green;
				displayCarLights(red);
				displayPedLights(green);
				
				pedMsg = OSMboxAccept(PedMbox);
				carMsg = OSMboxAccept(CarMbox);
		
				if (carMsg != "CAR_REQUEST") {
					currentState = currentState;
				} else {
					nextState = red_flashingGreen;
					currentState = nextState;
				}
				break;
		
			case red_flashingGreen:
				carLights = red;
				pedLights = flashingGreen;
				displayCarLights(red);
				displayPedLights(flashingGreen);
				
				carMsg = OSMboxAccept(CarMbox);
				pedMsg = OSMboxAccept(PedMbox);
		
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
				carMsg = OSMboxAccept(CarMbox);
		
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






