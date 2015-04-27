
#include "chipid.h"

#define BATCH_SIZE 7		// always increase batchsize when adding new chipid

int check_id(unsigned int *chipid){

	unsigned int id[BATCH_SIZE][5]={{0, 202113296, 1398152743, 1325889908, 4110417923},
			{0, 202178320, 1398152743, 1325889908, 4110417921},
			{0, 369139736, 2920810598, 1366502558, 4110425664},
			{0, 385875974, 2920810598, 1366505196, 4110425666},
			{0, 385875975, 2920810598, 1366505196, 4110425669},
			{0, 385949704, 2920810598, 1366504697, 4110425664},
			{0, 352325649, 2919892232, 1355856886, 4110425668}

	};
	int i=0, j=0;
	char match=1;


	for(i=0; i<BATCH_SIZE; i++){
		match = 1;
		for(j=0; j<5; j++){
			if(id[i][j]!=chipid[j])match=0;
		}
		if(match==1)return 1;
	}

	return 0;

}
