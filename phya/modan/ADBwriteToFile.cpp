//
// ADBwriteToFile.cpp
//
// A utility for storing bulk debug output for later viewing.
//

#include <stdlib.h>
#include <stdio.h>

static bool newSheet = true;
static int count = 0;
FILE *fh;



void
ADBwriteToFile(char* s, int n)
{
	if (newSheet) {	
		fh = fopen("ADBfile.txt", "w");
		newSheet = false;
	}


	if (fh == NULL) return;


	fprintf(fh, "%s\n", s);

	count++;

	if (count == n) {
		fclose(fh);
		fh = NULL;
		newSheet = true;	// Next write will clear the file.
		count = 0;
		return;
	}
}



char*
ADBbar(int length)
{
	static char bar[256];

	int i;

	if (length > 255) length = 255;

	for(i=0; i< length; i++)
		bar[i] = '*';

	bar[i] = NULL;

	return bar;
}