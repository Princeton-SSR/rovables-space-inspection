/*
Low pass values for the ToF boards
*/
int INDEX_L = 0;
int VALUE_L = 0;
int SUM_L = 0;
int READINGS_L[WINDOW_SIZE_L];
int AVERAGED_L = 0;

int INDEX_M = 0;
int VALUE_M = 0;
int SUM_M = 0;
int READINGS_M[WINDOW_SIZE_M];
int AVERAGED_M = 0;

int INDEX_R = 0;
int VALUE_R = 0;
int SUM_R = 0;
int READINGS_R[WINDOW_SIZE_R];
int AVERAGED_R = 0;


void LowPassFilterTof(){
    //Moving average filters on readings
    SUM_L = SUM_L - READINGS_L[INDEX_L];      // Remove the oldest entry from the sum
    VALUE_L = distance_l;
    if (VALUE_L > 500) { VALUE_L = 500; }     // Read the next sensor value, limit value for plots
    READINGS_L[INDEX_L] = VALUE_L;            // Add the newest reading to the window
    SUM_L = SUM_L + VALUE_L;                  // Add the newest reading to the sum
    INDEX_L = (INDEX_L + 1) % WINDOW_SIZE_L;  // Increment the index, and wrap to 0 if it exceeds the window size

    SUM_M = SUM_M - READINGS_M[INDEX_M];      // Remove the oldest entry from the sum
    VALUE_M = distance_m;
    if (VALUE_M > 500) { VALUE_M = 500; }     // Read the next sensor value, limit value for plots
    READINGS_M[INDEX_M] = VALUE_M;            // Add the newest reading to the window
    SUM_M = SUM_M + VALUE_M;                  // Add the newest reading to the sum
    INDEX_M = (INDEX_M + 1) % WINDOW_SIZE_M;  // Increment the index, and wrap to 0 if it exceeds the window size

    SUM_R = SUM_R - READINGS_R[INDEX_R];      // Remove the oldest entry from the sum
    VALUE_R = distance_r;
    if (VALUE_R > 500) { VALUE_R = 500; }     // Read the next sensor value, limit value for plots
    READINGS_R[INDEX_R] = VALUE_R;            // Add the newest reading to the window
    SUM_R = SUM_R + VALUE_R;                  // Add the newest reading to the sum
    INDEX_R = (INDEX_R + 1) % WINDOW_SIZE_R;  // Increment the index, and wrap to 0 if it exceeds the window size

    //set moving average distances
    distance_l = SUM_L / WINDOW_SIZE_L;
    distance_m = SUM_M / WINDOW_SIZE_M;
    distance_r = SUM_R / WINDOW_SIZE_R;
}