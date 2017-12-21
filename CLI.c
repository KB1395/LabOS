#include <readlight.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <termios.h>
#include <string.h>
//The purpose of this code is to control the light reading through commands
void main() {
	//The baud rate is an object "speed_t" added through the termios.h lib. The default value is 9600
	speed_t baud=B9600;
	//set of the port on which the Arduino is plugged. default value is "/dev/ttyACM0"
	char port[30];
	*port = "/dev/ttyACM0";
	//Here starts the loop that will wait for a command (exit with ctrl+C)
	while (1) {
		//char storing the keyboard entry
		char query[512];
		printf("Insert Command");
		//storing the keyboard entry
		scanf("%s", &query);
		//char used to parse the command in order to extract the parameters and values
		char** params;
		//split of the entry with a space as splitting character
		params = str_split(&query, " ");		
		//switch case about which command is entered
		if (strncmp(param[0],"help")==0){
			//help about the which commands are available
			printf('"set" allows to set the baud rate or the port');
			printf('"read" starts the program with the entered (or default if non entered) parameters');
		}
		if (strncmp(param[0],"set")==0){
			//command to set the baud rate and/or the port
			setfunc(&baud, &port, params);
		}

		if (strncmp(param[0],"read")==0){
			//command to start de reading of the arduino
			printf("%i \n", readlightmock(&port, baud));
		}
		if (strncmp(param[0],"exit")==0){
			exit(EXIT_SUCCESS);
		}
		else{
			printf("no such command");
		}

	}
}
//setfunc is used to recognise the parameters for the set command
void setfunc(char *baud, char *port, char **params) {
	switch (params[1]) {
	if (strncmp(param[0],"port")==0){
		//in case of port we take the value and save it in the port variable
		*port = params[2];
	}
	if (strncmp(param[0],"baud")==0){
		//in case of baud we take the int and save it in the baud variable
		*baud = stringtobaud(param[2]);
	}
	if (strncmp(param[0],"help")==0){
		//help for the help
		printf('"set port [portname]" to set port');
		printf('"set baud [baudrate]" to set the baud rate');
	}
	else{
		printf("no such parameter for the set command");
	}



}
//This function is used to transform a integer value set in the command into a baud rate object (speed_t)
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
//function used to split the command string in command, parameter and value
//source : https://stackoverflow.com/questions/9210528/split-string-with-delimiters-in-c
//I would have done it myself but i'm still strugguling with c.
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