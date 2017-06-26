/*
 * Silnik.h
 *
 *  Created on: 26 cze 2017
 *      Author: Erukolindo
 */

#include "main.h"
#ifndef SILNIK_H_
#define SILNIK_H_


typedef enum{ TYL, PRZOD} Kierunek;

typedef struct {
	uint32_t predkosc_; // w centymetrach/s
	Kierunek kierunek_;
	uint16_t moc_; //wype³nienie PWMa od 0 do 1000

}Silnik;

void Silnik_init(Silnik *sil)
{
	sil->predkosc_ = 0;
	sil->kierunek_ = PRZOD;
	sil->moc_ = 0;
}

void Silnik_setPredkosc(Silnik *sil, uint32_t predkosc)
{
	sil->predkosc_ = predkosc;
}
void Silnik_setMoc(Silnik *sil, uint16_t moc)
{
	if(moc > MAX_CCR_SILNIK)
		sil->moc_ = MAX_CCR_SILNIK;
	sil->moc_ = moc;

}
void Silnik_setKierunek(Silnik *sil, Kierunek kier)
{
	sil->kierunek_ = kier;
}

Kierunek Silnik_getKierunek(Silnik *sil)
{
	return sil->kierunek_;
}
uint32_t Silnik_getPredkosc(Silnik *sil)
{
	return sil->predkosc_;
}
uint16_t Silnik_getMoc(Silnik *sil)
{
	return sil->moc_;
}


#endif /* SILNIK_H_ */
