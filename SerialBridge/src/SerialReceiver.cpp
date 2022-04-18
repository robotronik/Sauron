#include <Arduino.h>
#include "data/SerialPacket.hpp"

SemaphoreHandle_t DataSemaphore;

const size_t buffersize = 1<<13;
size_t sizeused = 0;
char buffer[buffersize+1];

SerialTransmission LastReceived;

char* memmem(const char* haystack, size_t haystack_len, 
    const char* needle, size_t needle_len)
{
    if (haystack == NULL) return NULL; // or assert(haystack != NULL);
    if (haystack_len == 0) return NULL;
    if (needle == NULL) return NULL; // or assert(needle != NULL);
    if (needle_len == 0) return NULL;
    
    for (char* h = const_cast<char*>(haystack);
            haystack_len >= needle_len;
            ++h, --haystack_len) {
        if (!memcmp(h, needle, needle_len)) {
            return h;
        }
    }
    return NULL;
}

void SerialReceiveTask(void* params);

void StartSerialReceiver()
{
	DataSemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(DataSemaphore);
	xTaskCreate(SerialReceiveTask, "Serial Receiver", 10000, NULL, 1, NULL);
}

bool GetLastTransmission(SerialTransmission& data)
{
	if (xSemaphoreTake(DataSemaphore, 0) != pdTRUE)
	{
		return false;
	}
	data = LastReceived;
	xSemaphoreGive(DataSemaphore);
	return true;
}

void SerialReceiveTask(void* params)
{
	Serial.begin(SerialTransmission::BaudRate);
	memset(buffer, 0, buffersize +1);
	Serial.printf("Expected flags : STA = \"%s\", STO = \"%s\"", SerialTransmission::StartFlag.data(), SerialTransmission::StopFlag.data());
	for (;;)
	{
		size_t available = Serial.available();
		if (available > 0)
		{
			//Serial.printf("New data available : %d bytes\r\n", available);
			size_t sizeleft = buffersize - sizeused;
			if (available > sizeleft) //Move buffer if no space left
			{
				size_t overflow = sizeused + available - buffersize;
				memmove(&buffer[0], &buffer[overflow], sizeused);
				//Serial.printf("Buffer full, size at %d, shrinking %d bytes\r\n", sizeused, overflow);
				sizeused = sizeused - overflow;
			}
			Serial.readBytes(&buffer[sizeused], available);
			sizeused += available;
			/*Serial.printf("Read %d bytes, now at %d bytes used\r\n", available, sizeused);
			for (size_t i = 0; i < sizeused; i++)
			{
				Serial.printf("%02x ", buffer[i]);
			}
			Serial.println();
			for (size_t i = 0; i < sizeused; i++)
			{
				Serial.printf("%c", isAlphaNumeric(buffer[i]) ? buffer[i] : ' ' );
			}
			Serial.println();*/
			
			char* STApos = memmem(buffer, sizeused, SerialTransmission::StartFlag.data(), SerialTransmission::StartFlag.size()); //start of start flag
			char* STOpos = NULL, *startpos = NULL;//start of stop flag
			if (STApos != NULL)
			{
				startpos = &STApos[SerialTransmission::StartFlag.size()]; //end of start flag
				size_t left = sizeused - (startpos - buffer);
				STOpos = memmem(startpos, 
				left, SerialTransmission::StopFlag.data(), SerialTransmission::StopFlag.size());
			}
			//Serial.printf("STApos = %p, STOpos = %p \r\n", STApos, STOpos);
			if (STApos != NULL && STOpos != NULL && STApos < STOpos)
			{
				//Serial.printf("Found start at %p, stop at %p (size %d)\r\n", STApos, STOpos, STOpos-startpos);
				SerialTransmission transmission;
				if(transmission.FromBuffer(startpos, STOpos - STApos))
				{
					//Serial.printf("Decoded payload : ms=%d, NumPositions=%d, score = %d\r\n", transmission.ms, transmission.NumPositions, transmission.score);
					/*for (size_t i = 0; i < transmission.NumPositions; i++)
					{
						PositionPacket& pak = transmission.PositionPackets[i];
						Serial.printf("  Position %d, type %d, numeral %d, X %f, Y %f, rot %f\r\n", i, (int)pak.type, pak.numeral, pak.X, pak.Y, pak.rotation);
					}*/
					if (xSemaphoreTake(DataSemaphore, 100))
					{
						LastReceived = transmission;
						xSemaphoreGive(DataSemaphore);
					}
				}
				char* endpos = &STOpos[SerialTransmission::StopFlag.size()]; //end of stop flag
				size_t nextused = &buffer[sizeused] - endpos;
				memmove(buffer, endpos, nextused);
				memset(&buffer[nextused], 0, buffersize - nextused);
				sizeused = nextused;
				//Serial.printf("Now using %d bytes\r\n", sizeused);
			}
		}
		vTaskDelay(1);
	}
	

	vTaskDelete(NULL);
}