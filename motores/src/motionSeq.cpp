#include "ros/ros.h"
#include "dynamixel_sdk/dynamixel_sdk.h"     // Dynamixel SDK library
#include "usefull_functions.cpp"             // Extra functions: getch()  &  kbhit()
#include <iostream>
#include <stdlib.h>
#include <ctype.h>
#include <fstream>
#include <istream>
#include <dirent.h>
#include <string.h>
#include <sstream>

// 1.- Global variables
dynamixel::PortHandler *portHandler;         // Port handler
dynamixel::PacketHandler *packetHandler;     // Packet handler

int dxl_comm_result = COMM_TX_FAIL;          // Communication result
int dxl_comm_present = COMM_TX_FAIL;          // Communication result
int dxl_comm_present_mot = COMM_TX_FAIL;          // Communication result
int dxl_comm_goal = COMM_TX_FAIL;          // Communication result
int dxl_comm_goal_M = COMM_TX_FAIL;          // Communication result
uint8_t dxl_error = 0;                       // Dynamixel error
uint16_t dxl_model_number;                   // Dynamixel model number
uint16_t dxl_present_position = 0;               // Present position
uint16_t dxl_present_position_wrt = 0;               // Present position

int dxl_goal_position, dxl_goal_position_seq;
int position[18][1000];
int datosCrea[19] = {512, 651, 474, 446, 635, 426, 282, 302, 522, 0, 524, 377, 527, 595, 574, 567, 567, 567, 470};
int idM[20][100];
int connectM = 0;
int std_position[18] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int std_position_N[18][1000];
int each_position[18][1000];
int pres_position[18];
int pres_position_pos[18];
int write_position[18];
int move_position[18] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int move_position_sav[18] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int sequence_position[18][1000];
int idSelec, sizeM, sequen, sequenS, numFI, numCI, numEI;
size_t sizeD;
int noDatos = 0;
int afir = 0;
int afirPos = 0;
float t0;
float tf;
int q1[18];
int movimiento = 0;
int i = 0;
int j = 0;
int m = 0;
int movSpe = 60;
int movSpeed = 60;
using namespace std;
string idChan, id, file, fileChange, filePath, filePath_M, selMenu, sino, sinoPos, fileName, tiempo, dela, movMet, numFS, numCS, numES;
int dxl, dxlDZ, ddsxlDC, nav_d, n, dy, dxlDZ2, numMet, posEsc;
bool isNumber(string);
void guardDatos(string, string, string);
void guardDatos_2(string, string, string);
void finalP();
void finalP_2();
void savedD();
void saved2D();
void wrt();
void edit();
void leer();
void readfile(void){
	DIR *dir;
	struct dirent *dirp;
	if((dir = opendir("/home/bogobot21/catkin_ws/src/motores/src/sequences/")) == NULL){
		perror("\nNo se puede abrir la carpeta\n");
	}
	else{
		printf("\nLista se secuencias hechas:\n");
		while ((dirp = readdir(dir)) != NULL) {
			printf("%s\n", dirp->d_name);
		}
		closedir(dir);
	}
}
void dataMotor(void){
	cout << "\n¿Deseas seleccionar una sequencia ya creada? [Y/N]: ";
	cin >> sino;
	if(sino == "Y" || sino == "y"){
		afir = 1;
	}
	else if(sino == "N" || sino == "n"){
		afir = 2;
	}
	else if((sino != "N") || (sino != "n") || (sino != "Y") || (sino != "y")){
		printf("\nValor incorrecto, intente de nuevo\n");
	}
}

void torque_on(void) {
    for (int dxl_id = 0; dxl_id <= 18; ++dxl_id) {
	if(dxl_id == 9){
		continue;
	}
	// Sending Instruction Packet
	dxl_comm_result = packetHandler -> ping(portHandler, dxl_id, &dxl_model_number, &dxl_error);
	dxl_comm_goal = packetHandler->write1ByteTxRx(portHandler, dxl_id, 24, 1, &dxl_error);
	// Analizing received Status Packet
	if (dxl_comm_result != COMM_SUCCESS) // If comm is not succesful
		printf("El ID = %d no funciona correctamente, revisa su conexión\n", dxl_id);
	else if (dxl_error != 0)             // If dynamixel got an error
		printf("%s\n", packetHandler -> getRxPacketError(dxl_error));
	else                                 // If everything is ok
		++connectM;
    }
}

void presentP(void){
	if(connectM == 18){
		printf("\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> POSICIÓN ACTUAL >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
		for (int dxl_id = 0; dxl_id <= 18; ++dxl_id) {
			if(dxl_id == 9){
				continue;
			}
			dxl_comm_present = packetHandler->read2ByteTxRx(portHandler, dxl_id, 36, &dxl_present_position, &dxl_error);
			pres_position[dxl_id] = (int)dxl_present_position;
		        printf("ID: %i PresPos:%03d\n", dxl_id, pres_position[dxl_id]);
		}
    	}
	else{
		idSelec = 19;
	}
}

void menu(void){
	printf("\nMENÚ PRINCIPAL\n");
	printf("\tA. NUEVA SECUENCIA\n");
	printf("\tB. EDITAR SECUENCIA\n");
	printf("\tC. LEER SECUENCIA\n");
	cout << "\nEscribe la opción deseada(A, B o C) o escriba '9' para salir: ";
	cin >> selMenu;
	idSelec = atoi(selMenu.c_str());
	if(selMenu == "A" || selMenu == "a" || selMenu == "B" || selMenu == "b" || selMenu == "C" || selMenu == "c" || idSelec == 9){
		if(selMenu == "A" || selMenu == "a"){
			wrt();
		}
		if(selMenu == "B" || selMenu == "b"){
			edit();
		}
		if(selMenu == "C" || selMenu == "c"){
			leer();
		}
		if(idSelec == 9){
			idSelec = 19;	
		}
	}
	else{
		printf("\nSelecciona una opción válida\n");

	}



}

void edit(void){
	afir = 1;
	do{
	cout << "Escribe el nombre del archivo (sin la extensión): ";
	cin >> fileName;
	filePath_M = "catkin_ws/src/motores/src/sequences/" + fileName + ".txt";
	
	usleep(5000);
	ifstream datosServo;
	datosServo.open(filePath_M.c_str());

	if(!datosServo){
		cerr << "No se puede abrir el archivo\n" << endl;
		exit(1);
	}	
	while(!datosServo.eof()){
		datosServo >> idM[0][i];
		datosServo >> idM[1][i];
		datosServo >> idM[2][i];
		datosServo >> idM[3][i];
		datosServo >> idM[4][i];
		datosServo >> idM[5][i];
		datosServo >> idM[6][i];
		datosServo >> idM[7][i];
		datosServo >> idM[8][i];
		datosServo >> idM[10][i];
		datosServo >> idM[11][i];
		datosServo >> idM[12][i];
		datosServo >> idM[13][i];
		datosServo >> idM[14][i];
		datosServo >> idM[15][i];
		datosServo >> idM[16][i];
		datosServo >> idM[17][i];
		datosServo >> idM[18][i];
		datosServo >> idM[19][i];
		datosServo >> idM[20][i];
		
		++i; 
	}
	ddsxlDC = i;
	usleep(10000);
	cout << "Escribe el número la fila: ";
	cin >> numFS;
	numFI = atoi(numFS.c_str());
	cout << "Escribe el número la columa: ";
	cin >> numCS;
	numCI = atoi(numCS.c_str());
	printf("\nEl valor seleccionado es: %d\n",idM[numCI][numFI]);
	cout << "Escribe el nuevo valor: ";
	cin >> numES;
	numEI = atoi(numES.c_str());
	
	usleep(10000);
	idM[numCI][numFI] = numEI;
	printf("V: %d", idM[numCI][numFI]);
	ofstream guardaServos (filePath_M.c_str());
	for(int i = 0; i < ddsxlDC - 1; ++i){
		for (int dxl_id = 0; dxl_id <= 20; ++dxl_id){			
			if(dxl_id == 9){
				continue;
			}
			if(dxl_id == numCI && i == numFI){
				guardaServos << idM[numCI][numFI] << " ";
				continue;
			}
			guardaServos << idM[dxl_id][i] << " ";
		}
		guardaServos << "\n";
	}
	cout << "\n¿Deseas seguir editando? [Y/N]: ";
	cin >> sino;
	if(sino == "Y" || sino == "y"){
		afir = 1;
	}
	else if(sino == "N" || sino == "n"){
		idSelec = 19;
	}
	else if((sino != "N") || (sino != "n") || (sino != "Y") || (sino != "y")){
		printf("\nValor incorrecto, intente de nuevo\n");
	}
	}while(idSelec != 19);
	
}

void leer(void){
	cout << "Escribe el nombre del archivo (sin la extensión): ";
	cin >> fileName;
	filePath_M = "catkin_ws/src/motores/src/sequences/" + fileName + ".txt";
	
        savedD();
	saved2D();
	idSelec = 19;
}

void wrt(void){
	movimiento = 0;
	printf("\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> POSICIÓN A LLEGAR >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n\n");
	std::cout << "Escriba el ID del motor a mover o escriba '9' para salir: ";
	std::cin >> id;
	idSelec = atoi(id.c_str());	
	if(isNumber(id)){
		if((idSelec >= 0)||(idSelec <= 18)){
			if(idSelec == 9){
				idSelec = 19;	
			}
			else{
				std::cout << "¿Quieres realizar movimientos con teclas o escribir la posición?[1/2] ";
				std::cin >> movMet;
				numMet = atoi(movMet.c_str());	
				if(numMet == 1){
					dxl_comm_present = packetHandler->read2ByteTxRx(portHandler, idSelec, 36, &dxl_present_position, &dxl_error);
					write_position[idSelec] = (int)dxl_present_position;
					printf("ID: %d Present Position:%03d\n\n",idSelec, (int)dxl_present_position);
					cout << "\nUtiliza las teclas W/S para avanzar cada 20 unidades(+/-), E/D para avanzar cada 40 unidades(+/-), R/F para avanzar cada 80 unidades(+/-) o 'id' para cambiar de motor:\n";
					do{
						cin >> idChan;
						if(idChan == "W" || idChan == "w"){
							if(std_position[idSelec] == 0){
								std_position[idSelec] = 1;
							}
							else{
								std_position[idSelec] = std_position[idSelec] +1;
							}						
							dxl_goal_position = write_position[idSelec] + 20;
							printf("ID: %d Goal Position: %03d\n",idSelec, dxl_goal_position);
							dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, idSelec, 32, movSpe, &dxl_error);
							dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, idSelec, 30, dxl_goal_position, &dxl_error);
							write_position[idSelec] = dxl_goal_position;
							if(move_position[idSelec] >= movimiento){
								movimiento ++;
								move_position[idSelec] = movimiento + 1;
								sequence_position[idSelec][std_position[idSelec]] = dxl_goal_position;
								printf("mov: %d move_pos: %d volvio %d sequence: %d\n", movimiento, move_position[idSelec], std_position[idSelec], sequence_position[idSelec][std_position[idSelec]]);								
							}
							continue;
						}
						else if(idChan == "S" || idChan == "s"){
							if(std_position[idSelec] == 0){
								std_position[idSelec] = 1;
							}
							else{
								std_position[idSelec] = std_position[idSelec] +1;
							}	
							dxl_goal_position = write_position[idSelec] - 20;
							printf("ID: %d Goal Position: %03d\n",idSelec,dxl_goal_position);
							dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, idSelec, 32, movSpe, &dxl_error);
							dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, idSelec, 30, dxl_goal_position, &dxl_error);
							write_position[idSelec] = dxl_goal_position;
							if(move_position[idSelec] >= movimiento){
								movimiento ++;
								move_position[idSelec] = movimiento + 1;
								sequence_position[idSelec][std_position[idSelec]] = dxl_goal_position;
								printf("mov: %d move_pos: %d volvio %d sequence: %d\n", movimiento, move_position[idSelec], std_position[idSelec], sequence_position[idSelec][std_position[idSelec]]);						
							}
							continue;
						}
						else if(idChan == "E" || idChan == "e"){
							if(std_position[idSelec] == 0){
								std_position[idSelec] = 1;
							}
							else{
								std_position[idSelec] = std_position[idSelec] +1;
							}	
							dxl_goal_position = write_position[idSelec] + 40;
							printf("ID: %d Goal Position: %03d\n",idSelec,dxl_goal_position);
							dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, idSelec, 32, movSpe, &dxl_error);
							dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, idSelec, 30, dxl_goal_position, &dxl_error);
							write_position[idSelec] = dxl_goal_position;
							if(move_position[idSelec] >= movimiento){
								movimiento ++;
								move_position[idSelec] = movimiento + 1;
								sequence_position[idSelec][std_position[idSelec]] = dxl_goal_position;
								printf("mov: %d move_pos: %d volvio %d sequence: %d\n", movimiento, move_position[idSelec], std_position[idSelec], sequence_position[idSelec][std_position[idSelec]]);
							}
							continue;
						}
						else if(idChan == "D" || idChan == "d"){
							if(std_position[idSelec] == 0){
								std_position[idSelec] = 1;
							}
							else{
								std_position[idSelec] = std_position[idSelec] +1;
							}					
							dxl_goal_position = write_position[idSelec] - 40;
							printf("ID: %d Goal Position: %03d\n",idSelec,dxl_goal_position);
							dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, idSelec, 32, movSpe, &dxl_error);
							dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, idSelec, 30, dxl_goal_position, &dxl_error);
							write_position[idSelec] = dxl_goal_position;
							if(move_position[idSelec] >= movimiento){
								movimiento ++;
								move_position[idSelec] = movimiento + 1;
								sequence_position[idSelec][std_position[idSelec]] = dxl_goal_position;
								printf("mov: %d move_pos: %d volvio %d sequence: %d\n", movimiento, move_position[idSelec], std_position[idSelec], sequence_position[idSelec][std_position[idSelec]]);
							}
							continue;
						}
						else if(idChan == "R" || idChan == "r"){
							if(std_position[idSelec] == 0){
								std_position[idSelec] = 1;
							}
							else{
								std_position[idSelec] = std_position[idSelec] +1;
							}	
							dxl_goal_position = write_position[idSelec] + 80;
							printf("ID: %d Goal Position: %03d\n",idSelec,dxl_goal_position);
							dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, idSelec, 32, movSpe, &dxl_error);
							dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, idSelec, 30, dxl_goal_position, &dxl_error);
							write_position[idSelec] = dxl_goal_position;
							if(move_position[idSelec] >= movimiento){
								movimiento ++;
								move_position[idSelec] = movimiento + 1;
								sequence_position[idSelec][std_position[idSelec]] = dxl_goal_position;
								printf("mov: %d move_pos: %d volvio %d sequence: %d\n", movimiento, move_position[idSelec], std_position[idSelec], sequence_position[idSelec][std_position[idSelec]]);
							}
							continue;
						}
						else if(idChan == "F" || idChan == "f"){
							if(std_position[idSelec] == 0){
								std_position[idSelec] = 1;
							}
							else{
								std_position[idSelec] = std_position[idSelec] +1;
							}	
							dxl_goal_position = write_position[idSelec] - 80;
							printf("ID: %d Goal Position: %03d\n",idSelec,dxl_goal_position);
							dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, idSelec, 32, movSpe, &dxl_error);
							dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, idSelec, 30, dxl_goal_position, &dxl_error);
							write_position[idSelec] = dxl_goal_position;
							if(move_position[idSelec] >= movimiento){
								movimiento ++;
								move_position[idSelec] = movimiento + 1;
								sequence_position[idSelec][std_position[idSelec]] = dxl_goal_position;
								printf("mov: %d move_pos: %d volvio %d sequence: %d\n", movimiento, move_position[idSelec], std_position[idSelec], sequence_position[idSelec][std_position[idSelec]]);
							}
							continue;
						}
						else if(idChan != "id"){
							printf("Valor incorrecto, intente de nuevo\n");
						}
						else if(idChan == "id"){
							move_position_sav[idSelec] = std_position[idSelec];
							if(move_position_sav[0] < move_position_sav[idSelec]){
								move_position_sav[0] = move_position_sav[idSelec];
								dxlDZ2 = move_position_sav[idSelec];
								printf("El más grande: %d\n", dxlDZ2);
							}
							cout << "\n¿Deseas guardar la postura? [Y/N] (Si seleccionas que no, podrás continuar creando movimientos)\n";
							cin >> sinoPos;
							if(sinoPos == "Y" || sinoPos == "y"){
								if(afirPos == 0){
									std::cout << "Escriba el nombre del archivo para guardar la secuencia: ";
				   				 	std::cin >> file;
				    					filePath = "catkin_ws/src/motores/src/sequences/" + file + ".txt";
									std::cout << "Escriba el tiempo deseado para esta postura: ";
				   				 	std::cin >> tiempo;
									std::cout << "Escriba el delay deseado para esta postura: ";
				   				 	std::cin >> dela;
									finalP();
									guardDatos(filePath, tiempo, dela);
									afirPos = 1;
								}
								else{
									std::cout << "Escriba el tiempo deseado para esta postura: ";
				   				 	std::cin >> tiempo;
									std::cout << "Escriba el delay deseado para esta postura: ";
				   				 	std::cin >> dela;
									finalP_2();
									guardDatos_2(filePath, tiempo, dela);
								}
								fileChange = file;
							}
							else if(sinoPos == "N" || sinoPos == "n"){
								printf("\nContinua creando secuencias\n");
							}
							else if((sinoPos != "id")){
								printf("\nValor incorrecto, intente de nuevo\n");
							}
						}
		
					}while(idChan != "id");
				}
				else if(numMet == 2){
					dxl_comm_present = packetHandler->read2ByteTxRx(portHandler, idSelec, 36, &dxl_present_position, &dxl_error);
					write_position[idSelec] = (int)dxl_present_position;
					printf("ID: %d Present Position:%03d\n\n",idSelec, (int)dxl_present_position);
					cout << "\nEscribe la posición deseada o 'id' para cambiar de motor:\n";
					do{
						cin >> idChan;
						if(idChan != "id"){
							posEsc = atoi(idChan.c_str());
							if(std_position[idSelec] == 0){
								std_position[idSelec] = 1;
							}
							else{
								std_position[idSelec] = std_position[idSelec] +1;
							}						
							dxl_goal_position = posEsc;
							printf("ID: %d Goal Position: %03d\n",idSelec, dxl_goal_position);
							dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, idSelec, 32, movSpe, &dxl_error);
							dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, idSelec, 30, dxl_goal_position, &dxl_error);
							write_position[idSelec] = dxl_goal_position;
							if(move_position[idSelec] >= movimiento){
								movimiento ++;
								move_position[idSelec] = movimiento + 1;
								sequence_position[idSelec][std_position[idSelec]] = dxl_goal_position;
								printf("mov: %d move_pos: %d volvio %d sequence: %d\n", movimiento, move_position[idSelec], std_position[idSelec], sequence_position[idSelec][std_position[idSelec]]);								
							}
							continue;
		
						}
						else if(idChan != "id"){
							printf("Valor incorrecto, intente de nuevo\n");
						}
						else if(idChan == "id"){
							move_position_sav[idSelec] = std_position[idSelec];
							if(move_position_sav[0] < move_position_sav[idSelec]){
								move_position_sav[0] = move_position_sav[idSelec];
								dxlDZ2 = move_position_sav[idSelec];
								printf("El más grande: %d\n", dxlDZ2);
							}
							cout << "\n¿Deseas guardar la postura? [Y/N] (Si seleccionas que no, podrás continuar creando movimientos)\n";
							cin >> sinoPos;
							if(sinoPos == "Y" || sinoPos == "y"){
								if(afirPos == 0){
									std::cout << "Escriba el nombre del archivo para guardar la secuencia: ";
				   				 	std::cin >> file;
				    					filePath = "catkin_ws/src/motores/src/sequences/" + file + ".txt";
									std::cout << "Escriba el tiempo deseado para esta postura: ";
				   				 	std::cin >> tiempo;
									std::cout << "Escriba el delay deseado para esta postura: ";
				   				 	std::cin >> dela;
									finalP();
									guardDatos(filePath, tiempo, dela);
									afirPos = 1;
								}
								else{
									std::cout << "Escriba el tiempo deseado para esta postura: ";
				   				 	std::cin >> tiempo;
									std::cout << "Escriba el delay deseado para esta postura: ";
				   				 	std::cin >> dela;
									finalP_2();
									guardDatos_2(filePath, tiempo, dela);
								}
								fileChange = file;
							}
							else if(sinoPos == "N" || sinoPos == "n"){
								printf("\nContinua creando secuencias\n");
							}
							else if((sinoPos != "id")){
								printf("\nValor incorrecto, intente de nuevo\n");
							}
						}
		
					}while(idChan != "id");
				}
			}			
	
		}
		else{
			printf("\nValor inválido, intente nuevamente\n");
		}
	}
	else{
		printf("\nSe debe ingresar un valor tipo Integer, intete nuevamente\n");
	}
}

bool isNumber(string s){
	for(int i = 0; i < s.length(); i++)
		if(isdigit(s[i]) == false)
			return false;
	return true;
}

void finalP_2(void){
	for (int dxl_id = 0; dxl_id <= 18; ++dxl_id) {
		if(dxl_id == 9){
			continue;
		}
		dxl_comm_present = packetHandler->read2ByteTxRx(portHandler, dxl_id, 36, &dxl_present_position, &dxl_error);
		pres_position_pos[dxl_id] = (int)dxl_present_position;
		printf("ID: %i PresPos:%03d\n", dxl_id, pres_position_pos[dxl_id]);
	}
	for (int dxl_id = 0; dxl_id <= 18; ++dxl_id) {
		if(dxl_id == 9){
			continue;
		}
		sizeM = std_position[dxl_id];
		sequence_position[dxl_id][sizeM] = pres_position_pos[dxl_id];
	}	
		
	for (int dxl_id = 0; dxl_id <= 18; ++dxl_id) {
		if(dxl_id == 9){
			continue;
		}
		sizeM = std_position[dxl_id];
		printf("ID: %d sizeM: %d\n", dxl_id, sizeM);
		for (int nav = 0; nav <= sizeM; ++nav) {
			printf("ID: %d Positions: %d\n", dxl_id, sequence_position[dxl_id][nav]);
		}
	}
}

void finalP(void){
	//printf("\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> POSICIÓN FINAL >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
	for (int dxl_id = 0; dxl_id <= 18; ++dxl_id) {
		if(dxl_id == 9){
			continue;
		}
		sequence_position[dxl_id][0] = pres_position[dxl_id];
	}	
		
	for (int dxl_id = 0; dxl_id <= 18; ++dxl_id) {
		if(dxl_id == 9){
			continue;
		}
		sizeM = std_position[dxl_id];
		printf("ID: %d sizeM: %d\n", dxl_id, sizeM);
		for (int nav = 0; nav <= sizeM; ++nav) {
			printf("ID: %d Positions: %d\n", dxl_id, sequence_position[dxl_id][nav]);
		}
	}
}

void guardDatos(string filePath, string tiempo, string dela){

	int l;
	ofstream guardaServos (filePath.c_str());
	int nz = 1;
	for (int dxl_id = 0; dxl_id <= 18; ++dxl_id){			
		if(dxl_id == 9){
			continue;
		}
		guardaServos << sequence_position[dxl_id][0] << " ";
	}
	guardaServos << tiempo << " " << dela << " ";
	guardaServos << "\n";

	for (int dxl_id = 0; dxl_id <= 18; ++dxl_id) {			
		if(dxl_id == 9){
			continue;
		}
		sizeM = std_position[dxl_id];
		guardaServos << sequence_position[dxl_id][sizeM] << " ";
	}
	guardaServos << tiempo << " " << dela << " ";
	guardaServos << "\n";						
}

void guardDatos_2(string filePath, string tiempo, string dela){

	int l;
	ofstream guardaServos (filePath.c_str(), ios::app);
	int nz = 1;
	
	for (int dxl_id = 0; dxl_id <= 18; ++dxl_id) {			
		if(dxl_id == 9){
			continue;
		}
		sizeM = std_position[dxl_id];
		guardaServos << sequence_position[dxl_id][sizeM] << " ";
	}
	guardaServos << tiempo << " " << dela << " ";
	guardaServos << "\n";
}

void savedD(void){
	ifstream datosServo;
	datosServo.open(filePath_M.c_str());

	if(!datosServo){
		cerr << "No se puede abrir el archivo\n" << endl;
		exit(1);
	}	
	while(!datosServo.eof()){
		datosServo >> idM[0][i];
		datosServo >> idM[1][i];
		datosServo >> idM[2][i];
		datosServo >> idM[3][i];
		datosServo >> idM[4][i];
		datosServo >> idM[5][i];
		datosServo >> idM[6][i];
		datosServo >> idM[7][i];
		datosServo >> idM[8][i];
		datosServo >> idM[10][i];
		datosServo >> idM[11][i];
		datosServo >> idM[12][i];
		datosServo >> idM[13][i];
		datosServo >> idM[14][i];
		datosServo >> idM[15][i];
		datosServo >> idM[16][i];
		datosServo >> idM[17][i];
		datosServo >> idM[18][i];
		datosServo >> idM[19][i];
		datosServo >> idM[20][i];
		
		++i; 
	}
	ddsxlDC = i;
}

void saved2D(void){
	for (int i = 0; i < ddsxlDC - 1; ++i) {
		//Cuello y cabeza
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 18, 32, idM[19][i], &dxl_error);
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 18, 30, idM[18][i], &dxl_error);

		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 8, 32, idM[19][i], &dxl_error);
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 8, 30, idM[8][i], &dxl_error);		

		//Pierna
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 13, 32, idM[19][i], &dxl_error);
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 13, 30, idM[13][i], &dxl_error);

		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 3, 32, idM[19][i], &dxl_error);
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 3, 30, idM[3][i], &dxl_error);

		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 14, 32, idM[19][i], &dxl_error);
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 14, 30, idM[14][i], &dxl_error);

		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 4, 32, idM[19][i], &dxl_error);
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 4, 30, idM[4][i], &dxl_error);

		
		//Brazo
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 15, 32, idM[19][i], &dxl_error);
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 15, 30, idM[15][i], &dxl_error);

		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 5, 32, idM[19][i], &dxl_error);
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 5, 30, idM[5][i], &dxl_error);
		
		//Pierna
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 11, 34, 1023, &dxl_error);
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 11, 32, idM[19][i], &dxl_error);
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 11, 30, idM[11][i], &dxl_error);

		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 1, 34, 1023, &dxl_error);
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 1, 32, idM[19][i], &dxl_error);
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 1, 30, idM[1][i], &dxl_error);

		//Pierna
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 12, 34, 1023, &dxl_error);
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 12, 32, idM[19][i], &dxl_error);
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 12, 30, idM[12][i], &dxl_error);

		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 2, 34, 1023, &dxl_error);
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 2, 32, idM[19][i], &dxl_error);
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 2, 30, idM[2][i], &dxl_error);

		//Pierna
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 10, 32, idM[19][i], &dxl_error);
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 10, 30, idM[10][i], &dxl_error);

		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 0, 32, idM[19][i], &dxl_error);
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 0, 30, idM[0][i], &dxl_error);

		//Brazo
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 16, 32, idM[19][i], &dxl_error);
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 16, 30, idM[16][i], &dxl_error);

		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 6, 32, idM[19][i], &dxl_error);
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 6, 30, idM[6][i], &dxl_error);

		//Brazo
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 17, 32, idM[19][i], &dxl_error);
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 17, 30, idM[17][i], &dxl_error);

		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 7, 32, idM[19][i], &dxl_error);
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 7, 30, idM[7][i], &dxl_error);

		usleep(idM[20][i]*1000);
	}


	idSelec = 19;

}

void motionP(void){
	int zer = 0;
	int pos;
	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 17, 32, movSpeed, &dxl_error);
	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 17, 30, pres_position[17], &dxl_error);

	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 7, 32, movSpeed, &dxl_error);
	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 7, 30, pres_position[7], &dxl_error);

	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 14, 32, movSpeed, &dxl_error);
	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 14, 30, pres_position[14], &dxl_error);

	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 4, 32, movSpeed, &dxl_error);
	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 4, 30, pres_position[4], &dxl_error);
	
	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 13, 32, movSpeed, &dxl_error);
	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 13, 30, pres_position[13], &dxl_error);

	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 3, 32, movSpeed, &dxl_error);
	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 3, 30, pres_position[3], &dxl_error);

	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 12, 32, movSpeed, &dxl_error);
	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 12, 30, pres_position[12], &dxl_error);

	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 2, 32, movSpeed, &dxl_error);
	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 2, 30, pres_position[2], &dxl_error);

	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 11, 32, movSpeed, &dxl_error);
	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 11, 30, pres_position[11], &dxl_error);

	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 1, 32, movSpeed, &dxl_error);
	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 1, 30, pres_position[1], &dxl_error);

	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 10, 32, movSpeed, &dxl_error);
	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 10, 30, pres_position[10], &dxl_error);

	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 0, 32, movSpeed, &dxl_error);
	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 0, 30, pres_position[0], &dxl_error);

	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 15, 32, movSpeed, &dxl_error);
	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 15, 30, pres_position[15], &dxl_error);

	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 5, 32, movSpeed, &dxl_error);
	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 5, 30, pres_position[5], &dxl_error);

	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 16, 32, movSpeed, &dxl_error);
	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 16, 30, pres_position[16], &dxl_error);

	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 6, 32, movSpeed, &dxl_error);
	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 6, 30, pres_position[6], &dxl_error);

	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 18, 32, movSpeed, &dxl_error);
	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 18, 30, pres_position[18], &dxl_error);

	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 8, 32, movSpeed, &dxl_error);
	dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, 8, 30, pres_position[8], &dxl_error);


}

void torque_off(void) {    // Disable Dynamixel Torque
    // Sending Instruction Packet
    printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> DESHABILITAR TORQUE >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
    for (int dxl_id = 0; dxl_id <= 8; ++dxl_id) {
	    dxl_comm_goal = packetHandler->write1ByteTxRx(portHandler, dxl_id, 24, 0, &dxl_error);
	    // Analizing received Status Packet
	    if (dxl_comm_result != COMM_SUCCESS)
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	    else if (dxl_error != 0)
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	    else
		printf("Torque del ID: %d deshabilitado \n",dxl_id);
    }
    for (int dxl_id = 10; dxl_id <= 18; ++dxl_id) {
	    dxl_comm_goal = packetHandler->write1ByteTxRx(portHandler, dxl_id, 24, 0, &dxl_error);
	    // Analizing received Status Packet
	    if (dxl_comm_result != COMM_SUCCESS)
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	    else if (dxl_error != 0)
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	    else
		printf("Torque del ID: %d deshabilitado \n",dxl_id);
    }

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "std_pose");
    ros::NodeHandle n;

    // -------------------------- INICIA CODIGO DE DINAMIXELSDK ------------------------------------  
    portHandler = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0"); // Linux: "/dev/ttyUSB0" 
    packetHandler = dynamixel::PacketHandler::getPacketHandler(1.0);      // Protocol version
    
    portHandler->openPort();                 // 2.- Open port
    portHandler->setBaudRate(1000000);       // 3.- Set baudrate (Ex. 9600, 57600, 1000000)

    torque_on();
    presentP();
 
    readfile();

    if(idSelec != 19){
	    while(idSelec != 19){
		menu();                          // 4.- Read present position command
		sleep(1);
	    }
	    motionP();
	    printf("\nATENCIÓN: ASEGURA EL ROBOT, SE DESENERGIZAR EN 5 SEGUNDOS\n\n");
	    sleep(5);
	    torque_off();
    }
    else if(idSelec == 19){
	    printf("\nATENCIÓN: HUBO UN ERROR, SE DESABILITARAN LOS MOTORES EN 3 SEGUNDOS, ASEGURA EL ROBOT\n\n");
	    sleep(3);
	    torque_off();
    }
    
    portHandler->closePort();                // 5.- Close port
  
    // -------------------------- TERMINA CODIGO DE DINAMIXELSDK -----------------------------------
    ros::spinOnce();
    return 0;
}
