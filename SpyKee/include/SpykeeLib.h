#ifndef Spykee_h
#define Spykee_h
#include "PracticalSocket.h"
#include <stdio.h>

#define WRITEIMAGEONDISK

class Spykee
{
    	  public:
		enum operations
		{
			pharseNewMessage,
			takeChargeState,
			takeCameraPictureWithLock,
			unlockPicture
		};
		enum states
		{
			Connected = 1,
			Moving = 2,
			CameraOn = 4
		};
		Spykee(char*, char*);
		~Spykee();
		void unplug();
		void setplug();
		void move(int, int);
		void startCamera();
		unsigned char* captureFrame();
		int getState();
		void soundAllarm();
		void soundBomb();
		void turnOnCameraLight();
		void turnOffCameraLight();
             
             // La funzione pharseMessage analizza i messaggi, si salva le immagini contenute nei messaggi e le passa a
             // a chi le richiede. Gestisce anche il sincronismo tra chi passa i messaggi con le immagini e chi richiede
             // le immagini.
		static unsigned char* pharseMessage(unsigned char* m, int* lm, unsigned char ** image, enum operations op)
		{
			static int numImage = 0;
			static unsigned char image1[7000];
			static unsigned char image2[7000];
			static int lenImage1 = 0, lenImage2 = 0;
			static char blockImage1 = false, blockImage2 = false;
			static char lastSended = 2, toWrite  = 1;
			static int remaningLengthImage, lengthImage;
			static FILE* imm;
			
			if (op == takeCameraPictureWithLock)
			{
				if (lastSended == 1)
				{
				if (!blockImage2)
				{
					blockImage2 = true;
					lastSended = 2;
					*lm = lenImage2;
					if (image2 != NULL)
					{
						*image = image2;
					}
					return image2;
				}
				else
				{
					blockImage1 = true;
					*lm = lenImage1;
					if (image1 != NULL)
					{
						*image = image1;
					}
					return image1;
				}
				}
				else
				{
				if (!blockImage1)
				{
					blockImage1 = true;
					lastSended = 1;
					*lm = lenImage1;
					if (image1 != NULL)
					{
						*image = image1;
					}
					return image1;
				}
				else
				{
					blockImage2 = true;
					*lm = lenImage2;
					if (image2 != NULL)
					{
						*image = image2;
					}
					return image2;
				}
				}
			}
			
			if (op == pharseNewMessage)
			{
				int lenMessage = *lm;
				
				if ((m[0] == 80) && (m[1] == 75) && (m[2] == 2))
				{
				//Arrivata nuova immagine
					char name[] = {'i', 'm', 'm', 'x', 'x', 'x', '.', 'j', 'p', 'g', '\0'};
					int cc;
		
					remaningLengthImage = (m[3] << 8) + m[4];
					lengthImage = lenMessage - 5;
					
					//printf("\nDimensione iniziale = %d", remaningLengthImage);
					
					m += 5;
					if (toWrite == 1)
					{
						if (blockImage1)
						{
							remaningLengthImage = 0;
							return NULL;
						}
						printf("\nINITtt 1");
						blockImage1 = true;
						lenImage1 = remaningLengthImage;
						for (register int i = 0; i < lenMessage; i++)
						{
							image1[i] = m[i];
						}
					}
					else
					{
						if (blockImage2)
						{
							remaningLengthImage = 0;
							return NULL;
						}
						printf("\nINITtt 2");
						blockImage2 = true;
						lenImage2 = remaningLengthImage;
						for (register int i = 0; i < lenMessage; i++)
						{
							image2[i] = m[i];
						}
					}
					
					remaningLengthImage -= lenMessage - 5;
					
					numImage++;
					cc = numImage;
					name[5] = '0' + cc%10;
					cc /= 10;
					name[4] = '0' + cc%10;
					cc /= 10;
					name[3] = '0' + cc%10;
					//printf("\nArrivata nuova immagine %d", numImage);
					#ifdef WRITEIMAGEONDISK
					imm = fopen(name, "wb");
					fwrite(m, 1, (lenMessage - 5), imm);
					#endif
					//printf("\nDimensione nuova = %d, lunghezza pacchetto = %d", remaningLengthImage, lenMessage);
					
					return NULL;
				}
				
				if (remaningLengthImage > 0)
				{
				//Continuazione di un'immagine iniziata a ricevere precedentemente
				if (lenMessage > remaningLengthImage)
				{
					if (toWrite == 1)
					{
						for (register int i = 0, c = lengthImage; i < remaningLengthImage; i++, c++)
						{
							image1[c] = m[i];
						}
					}
					else
					{
						for (register int i = 0, c = lengthImage; i < remaningLengthImage; i++, c++)
						{
							image2[c] = m[i];
						}
					}
					
					#ifdef WRITEIMAGEONDISK
					fwrite(m, 1, remaningLengthImage, imm);
					fclose(imm);
					#endif
					//printf("\nPacchetto di %d, Scritti %d byte", lenMessage, remaningLengthImage);
					
					remaningLengthImage = 0;
					if (toWrite == 1)
					{
						blockImage1 = false;
						toWrite = 2;
					}
					else
					{
						blockImage2 = false;
						toWrite = 1;
					}
					
					lenMessage -= remaningLengthImage;
					Spykee::pharseMessage(m + remaningLengthImage, &lenMessage, NULL, pharseNewMessage);
					
					return NULL;
				}
				else
				{
					if (toWrite == 1)
					{
						for (register int i = 0, c = lengthImage; i < lenMessage; i++, c++)
						{
							image1[c] = m[i];
						}
					}
					else
					{
						for (register int i = 0, c = lengthImage; i < lenMessage; i++, c++)
						{
							image2[c] = m[i];
						}
					}
					
					lengthImage += lenMessage;
					remaningLengthImage -= lenMessage;
					
					//printf("\nDimensione nuova = %d, lunghezza pacchetto = %d", remaningLengthImage, lenMessage);
					#ifdef WRITEIMAGEONDISK
					fwrite(m, 1, lenMessage, imm);
					#endif
					if (remaningLengthImage <= 0)
					{
					if (toWrite == 1)
					{
						blockImage1 = false;
						toWrite = 2;
					}
					else
					{
						blockImage2 = false;
						toWrite = 1;
					}
					#ifdef WRITEIMAGEONDISK
					fclose(imm);
					#endif
					}
				}
				}
			}
			
			if (op == unlockPicture)
			{
				if (lastSended == 1)
				{
					blockImage1 = false;
					return NULL;
				}
				if (lastSended == 2)
				{
					blockImage2 = false;
					return NULL;
				}
			}
			
			//printf("\nRicevuto messaggio numero: %d", c);
			return NULL;
             }

      private:
		int state;
		TCPSocket* tcp;
};

#endif
