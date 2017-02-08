/*
 * Serwo.h
 *
 *  Created on: 7 lut 2017
 *      Author: Erukolindo
 */

#ifndef SERWO_H_
#define SERWO_H_

/*
 * Zalozenia
 *
 * Kat mozna regulowac co 1 stopien (czyli od 0 do 180)
 *
 *Kat mozna zmieniac tylko za pomoca fcji ustaw_kat
 *Analogicznie odczytywac
 *
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
	if(nowy_kat >180)
		nowy_kat = 180;
	serwo->kat = nowy_kat;
}

uint8_t Serwo_wartosc_kata(Serwo *serwo)
{
	return serwo->kat;
}


#endif /* SERWO_H_ */
