/*
 * Cyfrowy_Czujnik_Odleglosci.h
 *
 *  Created on: 3 lut 2017
 *      Author: Erukolindo
 */

#ifndef CYFROWY_CZUJNIK_ODLEGLOSCI_H_
#define CYFROWY_CZUJNIK_ODLEGLOSCI_H_

typedef enum{nie, tak} Bool;

typedef struct
{
	volatile Bool czy_jest_cos_widoczne;
}Cyfrowy_Czujnik_Odleglosci;

void Cyfrowy_Czujnik_Odleglosci_Init( Cyfrowy_Czujnik_Odleglosci *czujnik)
{
	czujnik->czy_jest_cos_widoczne = nie;
}

Bool Cyfrowy_Czujnik_Odleglosci_odczytaj_wskazanie(Cyfrowy_Czujnik_Odleglosci *czujnik)
{
	return czujnik->czy_jest_cos_widoczne;
}

void Cyfrowy_Czujnik_Odleglosci_ustaw_wskazanie(Cyfrowy_Czujnik_Odleglosci *czujnik, Bool wynik_pomiaru)
{
	czujnik->czy_jest_cos_widoczne = wynik_pomiaru;
}

#endif /* CYFROWY_CZUJNIK_ODLEGLOSCI_H_ */
