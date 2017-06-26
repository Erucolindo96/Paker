/*
 * Robot.h
 *
 *  Created on: 26 cze 2017
 *      Author: Erukolindo
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include "Cyfrowy_Czujnik_Odleglosci.h"
#include "Serwo.h"
#include "Analogowy_Czujnik_Odleglosci.h"

typedef struct {
Analogowy_Czujnik_Odleglosci czujnik_analogowy_;
Cyfrowy_Czujnik_Odleglosci czujnik_cyfrowy_lewy_, czujnik_cyfrowy_prawy_;
Serwo serwo_lewe_, serwo_prawe_;

}Robot;

void Robot_init(Robot *rob)
{
	Analogowy_Czujnik_Odleglosci_init(&(rob->czujnik_analogowy_));
	Cyfrowy_Czujnik_Odleglosci_Init(&(rob->czujnik_cyfrowy_lewy_));
	Cyfrowy_Czujnik_Odleglosci_Init(&(rob->czujnik_cyfrowy_prawy_));
	Serwo_init(&(rob->serwo_lewe_));
	Serwo_init(&(rob->serwo_prawe_));
}

void Robot_wymyslJakZareagowac(Robot *rob)
{


}

void Robot_zareaguj(Robot *rob)
{

}




#endif /* ROBOT_H_ */
