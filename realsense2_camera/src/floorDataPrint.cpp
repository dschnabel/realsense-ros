// compile with: g++ floorDataPrint.cpp -o floorDataPrint

#include <fstream>
#include <string.h>

#define FLOOR_MATRIX_COLS   422
#define FLOOR_MATRIX_ROWS   50

struct floorDataPair {
    double min;
    double max;
};

int main(int argc, char *argv[]) {
    if (argc < 2) {
        printf("no argument given\n");
        return -1;
    }

    bool min;
    if (strcmp(argv[1], "min") == 0) {
        min = true;
    } else if (strcmp(argv[1], "max") == 0) {
        min = false;
    } else {
        printf("wrong argument, must be either min or max\n");
        return -1;
    }

    std::ifstream floorDataFile("floorData.bin", std::ios::binary);

    if (floorDataFile.good()) {
        if (min) {
            printf("Minimum\n=======\n");
        } else {
            printf("Maximum\n=======\n");
        }
        for (int y = 0; y < FLOOR_MATRIX_ROWS; y++) {
            printf(",%d", y);
        }
        printf("\n");

        for (int x = 0; x < FLOOR_MATRIX_COLS; x++) {
            printf("%d", x);
            for (int y = 0; y < FLOOR_MATRIX_ROWS; y++) {
                struct floorDataPair pair;
                floorDataFile.read((char*)&pair, sizeof(pair));
                if (min) {
                    printf(",%f", pair.min);
                } else {
                    printf(",%f", pair.max);
                }
            }
            printf("\n");
        }
        floorDataFile.close();
    } else {
        printf("file error\n");
    }
   return 0;
}
