/***************************************************************************
                          artcomm.h  -  description
                             -------------------
    begin                : Sun Oct 1 2000
    copyright            : (C) 2000 by Cipriani Roberto
    email                : robocip@tiscalinet.it
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
#ifndef __ARTCOMM_H__
#define __ARTCOMM_H__

#define MSG_STARTSTOP			1
#define MSG_POSIZIONE			2
#define MSG_PERCEZIONE_PALLA		3
#define MSG_PERCEZIONE_ALTRI		4
#define MSG_UTILITA			5
#define MSG_MODULO			6	
#define MSG_GPS_PALLA			7	
#define MSG_GPS_GIOCATORI		8
#define MSG_SETGOPOS			9
#define MSG_KICKER			10


/***************************************************/
/* Header per la parte Dati dei messaggi scambiati */
/***************************************************/

struct MsgStartStop
{
unsigned char	Tipo;
unsigned char	Robot;
unsigned char	Formazione[4];
};

struct MsgPosizione
{
float		Posx;
float		Posy;
unsigned char	PosConf;
float		Theta;
unsigned char	ThetaConf;
float		Speed;
float		Jog;
unsigned char	VConf;
short		TimeStamp;
};

struct MsgPercezionePalla
{
float		Posx;
float		Posy;
unsigned char	PosConf;
float		Speed;
float		Theta;
unsigned char	VConf;
short		TimeStamp;
};

struct Giocatore
{
unsigned char	Id;
float		Posx;
float		Posy;
unsigned char	PosConf;
float		Theta;
unsigned char	ThetaConf;
float		Speed;
float		Jog;
unsigned char	VConf;
float		Dim;
unsigned char	DimConf;
short		TimeStamp;
};

struct MsgPercezioneAltri
{
unsigned char		Num;
struct Giocatore	Giocatori[1];
};

struct MsgSetGoPos
{
unsigned char	NumGiocatore;
char		Action;
float		Posx;
float		Posy;
float		Theta;
};

struct MsgKicker
{
unsigned char	NumGiocatore;
char		Action;
};

struct MsgUtilita
{
short		Schema;
char		Ruolo;
float		Val1;
float		Val2;
float		DistanzaPalla;
char		StalloRobot;
};

struct MsgModulo
{
char		Modulo;
};

/***************************************************/
/* Header per gli Schemi e Moduli                  */
/***************************************************/

struct Schema
{
char	Ruolo1;
char	Ruolo2;
char	Ruolo3;
char	Zona1;
char	Zona2;
char	Zona3;
char	FunUtil1;
char	FunUtil2;
};

struct Modulo
{
int			NumSchemi;
struct Schema		Schemi[1];
};

#endif

