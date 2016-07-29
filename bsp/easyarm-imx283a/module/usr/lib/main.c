#include <stdio.h>

int main(int argc, char **argv)
{
    int i,j;
    for (i=0; i<5; i++)
    {
        printf("main argc %d\n",argc);
        printf("shell:");
        for (j=0; j<argc; j++)
            printf("%s ",argv[j]);
        printf("\n");
    }
    return 0;
}
