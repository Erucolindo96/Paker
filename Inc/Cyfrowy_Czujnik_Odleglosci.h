/*
 * Cyfrowy_Czujnik_Odleglosci.h
 *
 *  Created on: 3 lut 2017
 *      Author: Erukolindo
 */

#ifndef CYFROWY_CZUJNIK_ODLEGLOSCI_H_
#define CYFROWY_CZUJNIK_ODLEGLOSCI_H_


typedef struct
{
	volatile enum{tak, nie} czy_jest_cos_widoczne;
}Cyfrowy_Czujnik_Odleglosci;

void Cyfrowy_Czujnik_Odleglosci_Init( Cyfrowy_Czujnik_Odleglosci *czujnik)
{
	czujnik->czy_jest_cos_widoczne = nie;
}



#endif /* CYFROWY_CZUJNIK_ODLEGLOSCI_H_ */
