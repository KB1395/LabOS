#include <readlight.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <termios.h>
void main() {
	speed_t baud=B9600;
	char port[30];
	port = "/dev/ttyACM0";
	while (1) {
		char query[512];

		char set_param[30];
		char set_value[256];

		printf("Insert Command");
		scanf("%s", &query);
		char** params;
		params=str_split(&query, " ");
		printf(query);
		
		switch (params[0]); {
		case "Help":
			printf('"set" allows to set the baud rate or the port');
			printf('"read" starts the program with the entered (or default if non entered) parameters');
		/*case "baud":
			printf("insert baud");
			char strbaud[50];
			scanf("%d", &strbaud);
			baud = atoi(strbaud);
		case "port":
			printf("insert port");
			scanf('%s', &port);*/
		case "set":
			setfunc(&baud, &port, params);

		case "read":
			printf("%i \n", readlightmock(&port, baud));

			
		}

	}
}

void setfunc(char *baud, char *port, char **params) {
	switch (params[1]) {
	case "port":
		*port = params[2];
	case "baud":
		*baud = stringtobaud(param[2]);
	case "help":
		printf('"set port [portname]" to set port');
		printf('"set baud [baudrate]" to set the baud rate')
	}



}
speed_t stringtobaud(char *strbaud) {
	int intbaud = atoi(strbaud);
	switch (intbaud) {
		case 0:
			return B0;
		case 50:
			return B50;
		case 75:
			return B75;
		case 110:
			return B110;
		case 134:
			return B134;
		case 150:
			return B150;
		case 200:
			return B200;
		case 300:
			return B300;
		case 600:
			return B600;
		case 1200:
			return B1200;
		case 1800:
			return B1800;
		case 2400:
			return B2400;
		case 4800:
			return B4800;
		case 9600:
			return B9600;
		case 19200:
			return B19200;
		case 38400:
			return B38400;
		default:
			return B9600;

	}
}
char** str_split(char* a_str, const char a_delim)
{
	char** result = 0;
	size_t count = 0;
	char* tmp = a_str;
	char* last_comma = 0;
	char delim[2];
	delim[0] = a_delim;
	delim[1] = 0;

	/* Count how many elements will be extracted. */
	while (*tmp)
	{
		if (a_delim == *tmp)
		{
			count++;
			last_comma = tmp;
		}
		tmp++;
	}

	/* Add space for trailing token. */
	count += last_comma < (a_str + strlen(a_str) - 1);

	/* Add space for terminating null string so caller
	knows where the list of returned strings ends. */
	count++;

	result = malloc(sizeof(char*) * count);

	if (result)
	{
		size_t idx = 0;
		char* token = strtok(a_str, delim);

		while (token)
		{
			assert(idx < count);
			*(result + idx++) = strdup(token);
			token = strtok(0, delim);
		}
		assert(idx == count - 1);
		*(result + idx) = 0;
	}

	return result;
}