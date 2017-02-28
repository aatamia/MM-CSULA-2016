#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#define STKSIZE 512
#define mSize 16

int ffcount = 0;
int push = 0;
int pop = 0;

const uint16_t NORTHMASK = 0b00000001;
const uint16_t EASTMASK = 0b00000010;
const uint16_t SOUTHMASK = 0b00000100;
const uint16_t WESTMASK = 0b00001000;
const uint16_t VISITMASK = 0x10;

int minOpenNeighbor = mSize*mSize - 2;
int distanceVal[16][16];

//Test Case A
//int16_t walls[16][16] = {{14,10,8,10,10,10,10,10,10,10,10,10,10,10,10,9},{12,10,2,10,10,10,10,10,10,10,10,10,10,10,9,5},{5,12,10,10,10,10,10,10,10,10,9,12,10,9,5,5},{5,5,12,9,12,9,14,8,10,9,6,0,11,5,5,5},{5,5,5,5,5,6,9,6,9,6,9,7,12,3,7,5},{5,5,5,6,2,11,5,12,2,11,6,10,3,12,9,5},{5,5,5,12,8,9,6,3,14,8,10,9,12,3,5,5},{5,5,6,3,7,5,13,12,9,5,12,3,6,9,5,5},{5,5,12,8,10,3,4,2,3,5,6,9,12,3,5,5},{5,5,5,6,9,12,1,12,9,5,12,3,6,9,5,5},{5,5,5,14,2,3,7,5,5,5,6,10,10,3,5,5},{5,5,6,9,12,10,10,3,5,6,10,9,12,9,5,5},{5,5,12,3,5,12,10,9,6,8,11,6,3,5,5,5},{5,5,6,10,3,5,14,1,14,0,9,12,9,5,5,5},{5,6,10,10,10,2,11,6,9,7,6,3,6,3,5,5},{6,10,10,10,10,10,10,10,2,10,10,10,10,10,2,3}};
//Test Case B
//int16_t walls[16][16] = {{14,9,12,10,9,14,8,10,11,13,14,10,8,11,13,13},{14,1,5,14,0,9,4,10,9,6,10,9,4,11,5,5},{14,1,4,11,5,5,5,13,5,14,10,1,5,14,1,5},{14,2,1,12,0,2,2,0,2,10,10,2,0,10,3,5},{13,12,3,5,6,9,13,5,13,13,13,13,7,12,10,1},{5,5,12,1,14,2,2,2,0,0,0,0,10,2,10,3},{6,3,5,6,11,12,8,11,7,5,7,6,8,10,11,13},{14,8,3,14,8,3,5,12,9,4,10,11,5,14,8,1},{14,2,8,8,1,13,7,4,3,5,12,11,6,9,7,5},{12,10,3,7,7,4,10,0,11,4,3,14,9,4,10,1},{5,14,8,11,12,3,12,0,11,4,10,11,4,3,13,5},{4,10,3,12,3,12,3,7,12,3,12,10,1,12,1,5},{4,11,12,3,12,3,13,12,2,9,5,13,6,3,4,1},{4,8,3,13,5,12,1,7,13,5,5,6,10,9,5,7},{5,6,10,2,2,3,4,9,5,6,0,8,9,4,0,11},{6,10,10,10,10,10,3,6,2,10,3,7,7,7,6,11}};
//Test Case D
int16_t walls[16][16] = {{14,10,8,10,10,10,10,10,10,10,10,10,10,10,10,9},{12,10,2,10,10,10,10,10,10,10,10,10,10,10,9,5},{5,12,10,9,12,10,10,10,9,12,10,10,10,9,5,5},{5,5,13,5,5,12,9,12,3,5,12,8,11,5,5,5},{5,5,5,5,5,5,6,3,13,6,3,6,9,5,5,5},{5,5,5,6,3,6,8,10,2,10,10,10,3,5,5,5},{5,6,2,10,10,9,6,10,10,10,10,9,12,1,5,5},{5,12,10,10,10,3,13,12,9,13,12,3,5,5,5,5},{5,5,13,13,12,10,1,6,1,4,3,12,3,7,5,5},{5,5,5,4,3,12,3,12,3,5,12,2,11,13,5,5},{5,5,4,3,12,3,12,1,12,3,6,10,9,4,1,5},{5,6,3,12,3,12,3,7,5,12,9,13,6,1,5,5},{5,13,12,3,12,0,10,9,5,5,5,4,9,5,5,5},{5,4,3,12,3,5,14,1,6,3,5,5,5,5,5,5},{5,6,10,2,10,2,10,2,10,11,6,3,6,3,5,5},{6,10,10,10,10,10,10,10,10,10,10,10,10,10,2,3}};
//Test Case C
//int16_t walls[16][16] = {{14,10,10,10,10,10,10,10,10,10,10,10,10,8,8,9},{12,10,10,8,8,9,12,9,12,8,9,13,13,7,5,7},{4,11,12,3,5,5,5,5,5,5,4,0,0,9,6,9},{4,11,6,9,5,6,3,6,3,4,3,7,5,6,9,5},{5,14,8,3,6,9,12,10,10,3,12,9,6,10,1,5},{5,12,1,12,10,3,4,9,12,10,3,6,10,9,5,5},{5,5,7,6,9,12,3,6,3,13,13,13,13,5,5,5},{5,6,9,12,1,6,9,12,8,0,0,0,0,1,5,5},{5,12,3,5,5,12,3,6,3,5,7,7,7,5,5,5},{5,5,12,2,3,6,9,12,9,6,9,12,10,3,5,5},{5,5,4,9,12,11,6,3,6,10,3,6,9,13,5,5},{5,4,3,6,2,10,9,12,11,12,10,10,3,4,3,5},{5,6,10,10,9,13,6,2,10,3,12,10,8,3,14,1},{4,10,8,11,6,2,10,10,10,10,3,13,5,12,10,3},{5,13,6,10,10,10,10,10,10,10,10,2,3,6,10,9},{6,2,10,10,10,10,10,10,10,10,10,10,10,10,10,3}};
int16_t currentDir = 0;
char dirTable[4] = {'^','>','v','<'};

int stksize = STKSIZE;
int currentCellX, currentCellY;

typedef struct coord {
    int x;
    int y;
}crd;

struct stack {
   crd stk[STKSIZE];
   int top;
}st;

void printDistanceValue(int16_t inputWall[mSize][mSize], int16_t col, int16_t row);
void printmaze();
void initialMaze();
void init_stk();
crd stkpop();
bool stkempty();
int smallerOpenCells(int x, int y);
void stkpush(crd val);
void ffConfigure();
void clearMaze();
void printMaze(int16_t inputWall[mSize][mSize]);
//void getWallsValue();

/*void getWallsValue(){
    int i;
    int j;
    int k;
    int l;
    walls[0][0] = 0b00001110;
    for (j = 1; j < 15; j++){
    	walls[0][j] = 0b00001000; //All West walls
    }
    walls[0][15] = 0b00001001; //North & West wall
    for (i = 1; i < 15; i++){
    	walls[i][15] = 0b00000001; //All North walls
    }
    walls[15][15] = 0b00000011; //North & East wall
    for (k = 1; k < 15; k++){
    	walls[15][k] = 0b00000010;// All East walls
    }
    walls[15][0] = 0b00000110; //South & East wall
    for (l = 1; l < 15; l++){
    	walls[l][0] = 0b00000100; //All South walls
    }
}*/

void init_stk() {
   st.top = -1;
}

int main(){
    system("COLOR F0");
	init_stk();
	setvbuf (stdout, NULL, _IONBF, 0);
	initialMaze();
	printmaze();
	printf("\n");

	int16_t NWO, EWO, SWO, WWO; //Check if Walls are open
 	int16_t PlusX, PlusY, MinusX, MinusY; //Check adj Cells
  	bool caseN, caseE, caseS, caseW; //Different cases
  	int16_t bs;


	while (distanceVal[currentCellX][currentCellY] != 0){
      NWO = walls[currentCellX][currentCellY] & NORTHMASK;
      EWO = walls[currentCellX][currentCellY] & EASTMASK;
      SWO = walls[currentCellX][currentCellY] & SOUTHMASK;
      WWO = walls[currentCellX][currentCellY] & WESTMASK;

      PlusY = currentCellY + 1; //Adj North
      PlusX = currentCellX + 1; //Adj East
      MinusY = currentCellY - 1; //Adj South
      MinusX = currentCellX - 1; //Adj West

      caseN = (NWO == 0x00) && (distanceVal[currentCellX][currentCellY] == (distanceVal[currentCellX][PlusY] + 1)) && (PlusY <= 15);
      caseE = (EWO == 0x00) && (distanceVal[currentCellX][currentCellY] == (distanceVal[PlusX][currentCellY] + 1)) && (PlusX <= 15);
      caseS = (SWO == 0x00) && (distanceVal[currentCellX][currentCellY] == (distanceVal[currentCellX][MinusY] + 1)) && (MinusY >= 0);
      caseW = (WWO == 0x00) && (distanceVal[currentCellX][currentCellY] == (distanceVal[MinusX][currentCellY] + 1)) && (MinusX >= 0);
      printMaze(walls);

      if (caseN){
      //means NORTH wall is open && less than 1
          currentCellY++;
          printMaze(walls);
      }
      else if (caseE){
      //means EAST wall is open && less than 1
          currentCellX++;
          printMaze(walls);
       }
       else if (caseS){
       //means SOUTH wall is open && less than 1
          currentCellY--;
          printMaze(walls);
       }
       else if (caseW){
       //means WEST wall is open && less than 1
          currentCellX--;
          printMaze(walls);
       }
       else {
           ffConfigure();

       }
	}
	printMaze(walls);
	printf("Amount of times Flood-Fill operation done: %d\n", ffcount);
	printf("Amount of times push operation called: %d\n", push);
	printf("Amount of times pop operation called: %d\n", pop);
	printf("\n\nThe End\n");


return 0;
}

int smallerOpenCells(int x, int y){
	int16_t north, south, east, west, counter;
	int Nopen, Eopen, Sopen, Wopen, i, j, leastVal;
	int leastNESW[4] = {255, 255, 255, 255};
	bool caseN, caseE, caseS, caseW, output;
	counter = 0;
	north = y + 1;
	east = x + 1;
	south = y - 1;
	west = x - 1;

	Nopen = walls[x][y] & NORTHMASK; //North Wall Mask Check
    Eopen = walls[x][y] & EASTMASK; 	//East Wall Mask Check
    Sopen = walls[x][y] & SOUTHMASK; //South Wall Mask Check
    Wopen = walls[x][y] & WESTMASK; 	//West Wall Mask Check

	caseN = (north <= 15) && (Nopen == 0) && ((distanceVal[x][y] - 1) == distanceVal[x][north]);
    caseE = (east <= 15) && (Eopen == 0) && ((distanceVal[x][y] - 1) == distanceVal[east][y]);
	caseS = (south >= 0) && (Sopen == 0) && ((distanceVal[x][y] - 1) == distanceVal[x][south]);
	caseW = (west >= 0) && (Wopen == 0) && ((distanceVal[x][y] - 1) == distanceVal[west][y]);

	if (caseN == 1){
		counter++;
	}
	else if ((north <= 15) && (Nopen == 0)){
        leastNESW[0] = distanceVal[x][north];
	}
	if (caseE == 1){
		counter++;
	}
    else if ((east <= 15) && (Eopen == 0)){
        leastNESW[1] = distanceVal[east][y] ;
	}
	if (caseS == 1){
		counter++;
	}
	else if ((south >= 0) && (Sopen == 0)){
        leastNESW[2] = distanceVal[x][south];
	}
	if (caseW == 1){
		counter++;
	}
	else if ((west >= 0) && (Wopen == 0)){
        leastNESW[3] = distanceVal[west][y];
	}
 /*   j = 0;
    while(j < 4){
        printf("LeastNESW: %d, ", leastNESW[j]);
        j++;
    }*/
	if (counter != 0){
		output = 1;
	}

	else {
        leastVal = leastNESW[0];
        for (i = 0; i < 3; i++){
            if(leastVal >= leastNESW[i+1]){
                leastVal = leastNESW[i+1];
                printf("least value: %d", leastVal);
            }
        }
        minOpenNeighbor = leastVal;
        printf("least value: %d", minOpenNeighbor);
        printf("\n");
		output = 0;
	}

return output;
}

void ffConfigure(){
	int anyopen;
    if (distanceVal[currentCellX][currentCellY] == 0){
        printf("Find more cells");
    }
    else {
        //Get wall values functions, for now we'll use a dummy global holding values
        int cellCoordXMM, cellCoordYMM, cellCoordXPP, cellCoordYPP;
        bool caseNW, caseSW, caseEW, caseWW;
        int left = currentCellX -  1;
        int right = currentCellX + 1;
        int up = currentCellY + 1;
        int down = currentCellY - 1;
        crd curr, adjN, adjE, adjS, adjW, cell_coord;
        crd openAdjN; //Open Adjacent Cell North
        crd openAdjE; //Open Adjacent Cell East
        crd openAdjS; //Open Adjacent Cell South
        crd openAdjW; //Open Adjacent Cell West

        curr.x = currentCellX;
        curr.y = currentCellY;
        adjN.x = currentCellX;
        adjN.y = up;
        adjE.x = right;
        adjE.y = currentCellY;
        adjS.x = currentCellX;
        adjS.y = down;
        adjW.x = left;
        adjW.y = currentCellY;


        stkpush(curr);
    /*
        if (adjN.y <= 15){stkpush(adjN);}
        if (adjE.x <= 15){stkpush(adjE);}
        if (adjW.x >= 0){stkpush(adjW);}
        if (adjS.y >= 0){stkpush(adjS);}
*/

        //getWallsValue()	;
        while (!stkempty()){
        	//printf("%d",st.top);
        	//printf("\n");
            cell_coord = stkpop();
            //printf("Mazevalue: %d", maze[cell_coord.x][cell_coord.y]);
            //printf("\n");
            int NWopen = walls[cell_coord.x][cell_coord.y] & NORTHMASK; //North Wall Mask Check
            int EWopen = walls[cell_coord.x][cell_coord.y] & EASTMASK; 	//East Wall Mask Check
            int SWopen = walls[cell_coord.x][cell_coord.y] & SOUTHMASK; //South Wall Mask Check
            int WWopen = walls[cell_coord.x][cell_coord.y] & WESTMASK; 	//West Wall Mask Check


            cellCoordYPP = cell_coord.y + 1;
            cellCoordXPP = cell_coord.x + 1;
            cellCoordYMM = cell_coord.y - 1;
            cellCoordXMM = cell_coord.x - 1;

            caseNW =  (cellCoordYPP <= 15) && (NWopen == 0) && (distanceVal[cell_coord.x][cell_coord.y] != 0) && ((distanceVal[cell_coord.x][cell_coord.y] - 1) != distanceVal[cell_coord.x][cellCoordYPP]);
            caseEW =  (cellCoordXPP <= 15) && (EWopen == 0) && (distanceVal[cell_coord.x][cell_coord.y] != 0) && ((distanceVal[cell_coord.x][cell_coord.y] - 1) != distanceVal[cellCoordXPP][cell_coord.y]);
            caseSW =  (cellCoordYMM >= 0) && (SWopen == 0) && (distanceVal[cell_coord.x][cell_coord.y] != 0) && ((distanceVal[cell_coord.x][cell_coord.y] - 1) != distanceVal[cell_coord.x][cellCoordYMM]);
            caseWW =  (cellCoordXMM >= 0) && (WWopen == 0) && (distanceVal[cell_coord.x][cell_coord.y] != 0) && ((distanceVal[cell_coord.x][cell_coord.y] - 1) != distanceVal[cellCoordXMM][cell_coord.y]);

            anyopen = smallerOpenCells(cell_coord.x, cell_coord.y);
/*
            printf("current cell x:%d, ", cell_coord.x);
            printf("current cell y:%d, ", cell_coord.y);
            printf("top of stack:%d, ", st.top);
            printf("cell distance popped:%d, ", distanceVal[cell_coord.x][cell_coord.y]);
            printf("North Adj value:%d, ", distanceVal[cell_coord.x][cellCoordYPP]);
            printf("East Adj value:%d, ", distanceVal[cellCoordXPP][cell_coord.y]);
            printf("South Adj value:%d, ", distanceVal[cell_coord.x][cellCoordYMM]);
            printf("West Adj value:%d, ", distanceVal[cellCoordXMM][cell_coord.y]);
            printf("min open cell:%d, ", minOpenNeighbor);
            printf("Check for any open cell < 1:%d\n ", anyopen);
*/
            //printf("%d\n", st.top);

           	if (anyopen == 0){
                distanceVal[cell_coord.x][cell_coord.y] = minOpenNeighbor + 1;
                ffcount++;
                    //printf("%d\n",(SWopen == 0));
           		if (caseNW){
           			//means NORTH wall is open && not less than 1
                	//distanceVal[cell_coord.x][cell_coord.y] = distanceVal[cell_coord.x][cellCoordYPP] + 1;
                	openAdjN.x = cell_coord.x;
                	openAdjN.y = cellCoordYPP;
                	stkpush(openAdjN);
            	}
           		if (caseEW){
                	//means EAST wall is open && not less than 1
                	//distanceVal[cell_coord.x][cell_coord.y] = distanceVal[cellCoordXPP][cell_coord.y] + 1;
                	openAdjE.x = cellCoordXPP;
                	openAdjE.y = cell_coord.y;
                	stkpush(openAdjE);
            	}
            	if (caseSW){
                	//means SOUTH wall is open && not less than 1
                	//distanceVal[cell_coord.x][cell_coord.y] = distanceVal[cell_coord.x][cellCoordYMM] + 1;
                	openAdjS.x = cell_coord.x;
                	openAdjS.y = cellCoordYMM;
                	stkpush(openAdjS);
            	}
            	if (caseWW){
                	//means WEST wall is open && not less than 1
                	//distanceVal[cell_coord.x][cell_coord.y] = distanceVal[cellCoordXMM][cell_coord.y] + 1;
                	openAdjW.x = cellCoordXMM;
                	openAdjW.y = cell_coord.y;
                	stkpush(openAdjW);
            	}
            }
            //clearStack();
        }
        clearStack();
    }
}

void stkpush(crd val) {
   st.top++;
   st.stk[st.top] = val;
   push++;
}

void clearStack(){
   st.top = -1;
}

crd stkpop() {
   crd val;
   val = st.stk[st.top];
   st.top--;
   pop++;
   return (val);
}

bool stkempty(){
   if (st.top == -1)
      return 1;
   else
      return 0;
}

void printmaze(){
    int i;
    int j;
    for (i = 0; i < 16; i++){
        printf(" ");
        printf("\n");
        for (j = 0; j < 16; j++){
            printf("%u ", distanceVal[i][j]);
        }
    }
}

void printMaze(int16_t inputWall[mSize][mSize])
{
    int16_t mazeSize = 16;
	int16_t col, row;
	for (col = mazeSize - 1; col >= 0; col--)
	{
		for (row = 0; row < mazeSize; row++)
		{
			if (inputWall[row][col] & NORTHMASK)//if has north wall
				printf("+----");
			else
				printf("+    ");
		}
		printf("+\r\n");
		for (row = 0; row < mazeSize; row++)
		{
			if (inputWall[row][col] & WESTMASK)//if has west wall
				printf("|");
			else
				printf(" ");
			printDistanceValue(inputWall, col, row);
		}
		if (inputWall[mazeSize-1][col] & EASTMASK)//if has east wall
			printf("|\r\n");
		else
			printf(" \r\n");
	}
	for (row = 0; row < mazeSize; row++)
	{
		if (inputWall[row][0] & SOUTHMASK)//if has south wall
			printf("+----");
		else
			printf("+    ");
	}
	printf("+\r\n");
}

void printDistanceValue(int16_t inputWall[mSize][mSize], int16_t col, int16_t row)
{
	int32_t val = distanceVal[row][col];

	if (row == currentCellX && col == currentCellY)
		printf("%c", dirTable[currentDir]);
	else
		printf(" ");

	if (val >= 4096)//if has 4 hex digits
		val %= 4096;
	printf("%3X", distanceVal[row][col]);
}

void initialMaze(){
  distanceVal[0][0] = 14;
  distanceVal[0][1] = 13;
  distanceVal[0][2] = 12;
  distanceVal[0][3] = 11;
  distanceVal[0][4] = 10;
  distanceVal[0][5] = 9;
  distanceVal[0][6] = 8;
  distanceVal[0][7] = 7;
  distanceVal[0][8] = 7;
  distanceVal[0][9] = 8;
  distanceVal[0][10] = 9;
  distanceVal[0][11] = 10;
  distanceVal[0][12] = 11;
  distanceVal[0][13] = 12;
  distanceVal[0][14] = 13;
  distanceVal[0][15] = 14;

  distanceVal[1][0] = 13;
  distanceVal[1][1] = 12;
  distanceVal[1][2] = 11;
  distanceVal[1][3] = 10;
  distanceVal[1][4] = 9;
  distanceVal[1][5] = 8;
  distanceVal[1][6] = 7;
  distanceVal[1][7] = 6;
  distanceVal[1][8] = 6;
  distanceVal[1][9] = 7;
  distanceVal[1][10] = 8;
  distanceVal[1][11] = 9;
  distanceVal[1][12] = 10;
  distanceVal[1][13] = 11;
  distanceVal[1][14] = 12;
  distanceVal[1][15] = 13;

  distanceVal[2][0] = 12;
  distanceVal[2][1] = 11;
  distanceVal[2][2] = 10;
  distanceVal[2][3] = 9;
  distanceVal[2][4] = 8;
  distanceVal[2][5] = 7;
  distanceVal[2][6] = 6;
  distanceVal[2][7] = 5;
  distanceVal[2][8] = 5;
  distanceVal[2][9] = 6;
  distanceVal[2][10] = 7;
  distanceVal[2][11] = 8;
  distanceVal[2][12] = 9;
  distanceVal[2][13] = 10;
  distanceVal[2][14] = 11;
  distanceVal[2][15] = 12;

  distanceVal[3][0] = 11;
  distanceVal[3][1] = 10;
  distanceVal[3][2] = 9;
  distanceVal[3][3] = 8;
  distanceVal[3][4] = 7;
  distanceVal[3][5] = 6;
  distanceVal[3][6] = 5;
  distanceVal[3][7] = 4;
  distanceVal[3][8] = 4;
  distanceVal[3][9] = 5;
  distanceVal[3][10] = 6;
  distanceVal[3][11] = 7;
  distanceVal[3][12] = 8;
  distanceVal[3][13] = 9;
  distanceVal[3][14] = 10;
  distanceVal[3][15] = 11;

  distanceVal[4][0] = 10;
  distanceVal[4][1] = 9;
  distanceVal[4][2] = 8;
  distanceVal[4][3] = 7;
  distanceVal[4][4] = 6;
  distanceVal[4][5] = 5;
  distanceVal[4][6] = 4;
  distanceVal[4][7] = 3;
  distanceVal[4][8] = 3;
  distanceVal[4][9] = 4;
  distanceVal[4][10] = 5;
  distanceVal[4][11] = 6;
  distanceVal[4][12] = 7;
  distanceVal[4][13] = 8;
  distanceVal[4][14] = 9;
  distanceVal[4][15] = 10;

  distanceVal[5][0] = 9;
  distanceVal[5][1] = 8;
  distanceVal[5][2] = 7;
  distanceVal[5][3] = 6;
  distanceVal[5][4] = 5;
  distanceVal[5][5] = 4;
  distanceVal[5][6] = 3;
  distanceVal[5][7] = 2;
  distanceVal[5][8] = 2;
  distanceVal[5][9] = 3;
  distanceVal[5][10] = 4;
  distanceVal[5][11] = 5;
  distanceVal[5][12] = 6;
  distanceVal[5][13] = 7;
  distanceVal[5][14] = 8;
  distanceVal[5][15] = 9;

  distanceVal[6][0] = 8;
  distanceVal[6][1] = 7;
  distanceVal[6][2] = 6;
  distanceVal[6][3] = 5;
  distanceVal[6][4] = 4;
  distanceVal[6][5] = 3;
  distanceVal[6][6] = 2;
  distanceVal[6][7] = 1;
  distanceVal[6][8] = 1;
  distanceVal[6][9] = 2;
  distanceVal[6][10] = 3;
  distanceVal[6][11] = 4;
  distanceVal[6][12] = 5;
  distanceVal[6][13] = 6;
  distanceVal[6][14] = 7;
  distanceVal[6][15] = 8;

  distanceVal[7][0] = 7;
  distanceVal[7][1] = 6;
  distanceVal[7][2] = 5;
  distanceVal[7][3] = 4;
  distanceVal[7][4] = 3;
  distanceVal[7][5] = 2;
  distanceVal[7][6] = 1;
  distanceVal[7][7] = 0;
  distanceVal[7][8] = 0;
  distanceVal[7][9] = 1;
  distanceVal[7][10] = 2;
  distanceVal[7][11] = 3;
  distanceVal[7][12] = 4;
  distanceVal[7][13] = 5;
  distanceVal[7][14] = 6;
  distanceVal[7][15] = 7;

  distanceVal[8][0] = 7;
  distanceVal[8][1] = 6;
  distanceVal[8][2] = 5;
  distanceVal[8][3] = 4;
  distanceVal[8][4] = 3;
  distanceVal[8][5] = 2;
  distanceVal[8][6] = 1;
  distanceVal[8][7] = 0;
  distanceVal[8][8] = 0;
  distanceVal[8][9] = 1;
  distanceVal[8][10] = 2;
  distanceVal[8][11] = 3;
  distanceVal[8][12] = 4;
  distanceVal[8][13] = 5;
  distanceVal[8][14] = 6;
  distanceVal[8][15] = 7;

  distanceVal[9][0] = 8;
  distanceVal[9][1] = 7;
  distanceVal[9][2] = 6;
  distanceVal[9][3] = 5;
  distanceVal[9][4] = 4;
  distanceVal[9][5] = 3;
  distanceVal[9][6] = 2;
  distanceVal[9][7] = 1;
  distanceVal[9][8] = 1;
  distanceVal[9][9] = 2;
  distanceVal[9][10] = 3;
  distanceVal[9][11] = 4;
  distanceVal[9][12] = 5;
  distanceVal[9][13] = 6;
  distanceVal[9][14] = 7;
  distanceVal[9][15] = 8;

  distanceVal[10][0] = 9;
  distanceVal[10][1] = 8;
  distanceVal[10][2] = 7;
  distanceVal[10][3] = 6;
  distanceVal[10][4] = 5;
  distanceVal[10][5] = 4;
  distanceVal[10][6] = 3;
  distanceVal[10][7] = 2;
  distanceVal[10][8] = 2;
  distanceVal[10][9] = 3;
  distanceVal[10][10] = 4;
  distanceVal[10][11] = 5;
  distanceVal[10][12] = 6;
  distanceVal[10][13] = 7;
  distanceVal[10][14] = 8;
  distanceVal[10][15] = 9;

  distanceVal[11][0] = 10;
  distanceVal[11][1] = 9;
  distanceVal[11][2] = 8;
  distanceVal[11][3] = 7;
  distanceVal[11][4] = 6;
  distanceVal[11][5] = 5;
  distanceVal[11][6] = 4;
  distanceVal[11][7] = 3;
  distanceVal[11][8] = 3;
  distanceVal[11][9] = 4;
  distanceVal[11][10] = 5;
  distanceVal[11][11] = 6;
  distanceVal[11][12] = 7;
  distanceVal[11][13] = 8;
  distanceVal[11][14] = 9;
  distanceVal[11][15] = 10;

  distanceVal[12][0] = 11;
  distanceVal[12][1] = 10;
  distanceVal[12][2] = 9;
  distanceVal[12][3] = 8;
  distanceVal[12][4] = 7;
  distanceVal[12][5] = 6;
  distanceVal[12][6] = 5;
  distanceVal[12][7] = 4;
  distanceVal[12][8] = 4;
  distanceVal[12][9] = 5;
  distanceVal[12][10] = 6;
  distanceVal[12][11] = 7;
  distanceVal[12][12] = 8;
  distanceVal[12][13] = 9;
  distanceVal[12][14] = 10;
  distanceVal[12][15] = 11;

  distanceVal[13][0] = 12;
  distanceVal[13][1] = 11;
  distanceVal[13][2] = 10;
  distanceVal[13][3] = 9;
  distanceVal[13][4] = 8;
  distanceVal[13][5] = 7;
  distanceVal[13][6] = 6;
  distanceVal[13][7] = 5;
  distanceVal[13][8] = 5;
  distanceVal[13][9] = 6;
  distanceVal[13][10] = 7;
  distanceVal[13][11] = 8;
  distanceVal[13][12] = 9;
  distanceVal[13][13] = 10;
  distanceVal[13][14] = 11;
  distanceVal[13][15] = 12;

  distanceVal[14][0] = 13;
  distanceVal[14][1] = 12;
  distanceVal[14][2] = 11;
  distanceVal[14][3] = 10;
  distanceVal[14][4] = 9;
  distanceVal[14][5] = 8;
  distanceVal[14][6] = 7;
  distanceVal[14][7] = 6;
  distanceVal[14][8] = 6;
  distanceVal[14][9] = 7;
  distanceVal[14][10] = 8;
  distanceVal[14][11] = 9;
  distanceVal[14][12] = 10;
  distanceVal[14][13] = 11;
  distanceVal[14][14] = 12;
  distanceVal[14][15] = 13;

  distanceVal[15][0] = 14;
  distanceVal[15][1] = 13;
  distanceVal[15][2] = 12;
  distanceVal[15][3] = 11;
  distanceVal[15][4] = 10;
  distanceVal[15][5] = 9;
  distanceVal[15][6] = 8;
  distanceVal[15][7] = 7;
  distanceVal[15][8] = 7;
  distanceVal[15][9] = 8;
  distanceVal[15][10] = 9;
  distanceVal[15][11] = 10;
  distanceVal[15][12] = 11;
  distanceVal[15][13] = 12;
  distanceVal[15][14] = 13;
  distanceVal[15][15] = 14;
}
