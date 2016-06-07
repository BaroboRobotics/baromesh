#include "baromesh/linkbot.h"
#include <stdio.h>

int main (int argc, char** argv) {
    if (argc < 3) {
        printf("Usage: %s <host> <port>\n", argv[0]);
        return 1;
    }

    printf("Connecting to %s:%s ... ", argv[1], argv[2]);
    if (baromesh::Linkbot* l = linkbotFromTcpEndpoint(argv[1], argv[2])) {
    	printf("Success!\n");

    	char serialId[5];
    	if (!linkbotGetSerialId(l, serialId)) {
    		printf("Serial ID is %s\n", serialId);
    	}
    	else {
    		printf("Failed to get serial ID\n");
    	}

    	linkbotDelete(l);
    }
    else {
    	printf("Failure!\n");
    }
}
