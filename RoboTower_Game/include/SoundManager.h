/*
 * RoboTower, Hi-CoRG based on ROS
 *
 * Copyright (C) 2012 Politecnico di Milano
 * Copyright (C) 2012 Marcello Pogliani, Davide Tateo
 * Versione 1.0
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef SOUNDMANAGER_H_
#define SOUNDMANAGER_H_

#include <QMediaPlayer>
#include <map>

class SoundManager
{

public:
	enum Sound
	{
		Start, Win, Lose, Trapped, Recharged
	};

public:
	SoundManager() :
			player(NULL, QMediaPlayer::LowLatency)
	{
		sounds[Start] = getSound("start.wav");
		sounds[Win] = getSound("win.wav");
		sounds[Lose] = getSound("lose.wav");
		sounds[Trapped] = getSound("trapped.wav");
		sounds[Recharged] = getSound("recharged.wav");
	}

	inline void play(Sound event)
	{
		player.setMedia(sounds[event]);
		player.play();
	}

	inline QMediaContent getSound(QString soundName)
	{
		QString baseDir = QCoreApplication::applicationDirPath();
		QString soundPath = baseDir + "/../sound/" + soundName;
		return QMediaContent(QUrl::fromLocalFile(soundPath));
	}

private:
	std::map<Sound, QMediaContent> sounds;
	QMediaPlayer player;

};

#endif /* SOUNDMANAGER_H_ */
