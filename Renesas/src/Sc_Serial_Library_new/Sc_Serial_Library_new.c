#include "Sc_Serial_Library_new.h"

//tools
static int strlen(uint8_t template[])
{
	int count = 0;
	while(template[count] != '\0')
		count++;
	return count;
}

void Bluetooth_Writestring(uint8_t string[])
{
	int length = strlen(string);
	if(length>0)
	{
		SCI5_Serial_Send("MavLink:",8);
		SCI5_Serial_Send(string,length);
		SCI5_Serial_Send("\n",1);
		//delay_ms(100);
		//SCI5_Serial_Send("\n",1);
	}
}

int my_printf(const char *fmt, ...)
{
	char buf[100];
	uint8_t buf2[100];
	int i;

    va_list args;
    int n;

    va_start(args, fmt);
    n = vsprintf(buf, fmt, args);
    va_end(args);

    //strlen(buf);

    for(i=0;i<n;i++)
    {
    	buf2[i] = buf[i];
    }

    SCI5_Serial_Send(buf2,n);
    //SCI5_Serial_Receive(buf2,1);

    return n;
}

void mavlink_printf(const char *fmt, ...)
{
	char buf[100];
	uint8_t buf2[100];
	int i;

	va_list args;
	int n;

	va_start(args, fmt);
	n = vsprintf(buf, fmt, args);
	va_end(args);

	//strlen(buf);

	for(i=0;i<n;i++)
	{
		buf2[i] = buf[i];
	}
	my_printf("MavLink:");
	SCI5_Serial_Send(buf2,n);
	my_printf("\n");
	//SCI5_Serial_Receive(buf2,1);

	//return n;
}
