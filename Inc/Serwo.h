/*
 * Serwo.h
 *
 *  Created on: 7 lut 2017
 *      Author: Erukolindo
 */

#ifndef SERWO_H_
#define SERWO_H_

/**
 * Zalozenia
 *
 * Kat mozna regulowac co 1 stopien (czyli od 0 do 180)
 *
 *Kat mozna zmieniac tylko za pomoca fcji ustaw_kat
 *Analogicznie odczytywac
 *Jest tez fcja do obliczania wartosci CCR, ktora nalezy ustawic, by uzyskac zadany kat
*/
typedef struct
{
	uint8_t kat;
}Serwo;

void Serwo_init(Serwo *serwo)
{
	serwo->kat = 0;
}

void Serwo_ustaw_nowy_kat(Serwo *serwo, uint8_t nowy_kat)
{
	if(nowy_kat > 180)
		nowy_kat = 180;
	serwo->kat = nowy_kat;
}

uint8_t Serwo_wartosc_kata(Serwo *serwo)
{
	return serwo->kat;
}

uint16_t Serwo_oblicz_wartosc_CCR(uint8_t kat)
{
	uint16_t wynik = (458 * kat)/100 + 300;
	return wynik;
}

#endif /* SERWO_H_ */
