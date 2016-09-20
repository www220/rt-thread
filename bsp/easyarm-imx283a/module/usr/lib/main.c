#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>

void alarm_ha(int sig)
{
	static int count = 0;
	printf("alarm  har %03d\n",count++);
	alarm(5);
}
int main(int argc, char **argv)
{
	int i;
	alarm(5);
	signal(SIGALRM, alarm_ha);
	for (i=0; i<600; i++)
	{
		printf("sleep %03d\n",i);
		sleep(1);
	}
    return 0;
}
