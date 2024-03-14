void classifierAlgorithm();

int32_t magAccVals[25];
int32_t filteredVals[25];
int16_t filterCoeffs[25] = {-22, -26, -33, -30, 0, 75, 207, 396, 626, 866, 1078, 1224, 1276, 1224, 1078, 866, 626, 396, 207, 75, 0, -30, -33, -26, -22 };
uint16_t beatCount;
int32_t beatVals[30];
int16_t beatDelays[30];
uint16_t lastBeatTime;
uint16_t delayIndex = 0; 
int32_t threshold = 50000000;


uint16_t bins[10];
uint16_t binremainders[10];
int16_t bin_thresholds[11] = {2000, 857, 750, 667, 600, 545, 500, 462, 429, 400, 300};