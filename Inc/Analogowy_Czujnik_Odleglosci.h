/*
 * Analogowy_Czujnik_Odleglosci.h
 *
 *  Created on: 11 sty 2017
 *      Author: Erukolindo
 */

#ifndef ANALOGOWY_CZUJNIK_ODLEGLOSCI_H_
#define ANALOGOWY_CZUJNIK_ODLEGLOSCI_H_

typedef enum {brak, pierwsza, druga, trzecia} Faza_Pomiaru;

typedef struct Analogowy_Czujnik_Odleglosci
{
	volatile u_int16_t odleglosc;//w milimetrach
	volatile Faza_Pomiaru faza_pomiaru;
}Analogowy_Czujnik_Odleglosci;

void Analogowy_Czujnik_Odleglosci_init(Analogowy_Czujnik_Odleglosci *czujnik)
{
	czujnik->odleglosc = 0;
	czujnik->faza_pomiaru = brak;
}

Faza_Pomiaru Analogowy_Czujnik_Odleglosci_faza_pomiaru(Analogowy_Czujnik_Odleglosci *czujnik)
{
	return czujnik->faza_pomiaru;
}
uint16_t Analogowy_Czujnik_Odleglosci_odleglosc(Analogowy_Czujnik_Odleglosci *czujnik)
{
	return czujnik->odleglosc;
}
uint16_t Analogowy_Czujnik_Odleglosci_oblicz_odleglosc(uint16_t wynik_timera, uint16_t prescaler, uint16_t ARR)
{
	return (wynik_timera * prescaler * 265)/100000;//obliczony na podstawie zegara 64Mhz i ARR = 0xFFFF
}


#endif /* ANALOGOWY_CZUJNIK_ODLEGLOSCI_H_ */
