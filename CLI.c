#include <readardserial.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
void main() {
	int baud;
	baud= 9600;
	char port[30];
	port = "/dev/ttyACM0";
	while (1) {
		char query[512];

		char set_param[30];
		char set_value[256];

		printf("Insert Command");
		scanf("%s", &query);
		params=str_split(&query, " ");
		printf(query);
		
		switch (params[0]); {
		case "Help":
			printf('"baud" allows to set the baud rate fro the serial port (default 9600)');
			printf('"serial" allows to set the serial port (default /dev/ttyACM0)');
			printf('"read" starts the program with the entered (or default if non entered) parameters');
		case "baud":
			printf("insert baud");
			char strbaud[50];
			scanf("%d", &strbaud);
			baud = atoi(strbaud);
		case "port":
			printf("insert port");
			scanf('%s', &port);
		case "set":
			setfunc(&set_param, &set_value, params);

		case "read":


			
		}

	}
}

void setfunc(char *out_param, char *out_value, char **params) {
	*out_param = params[1];
	*out_value = params[2];



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