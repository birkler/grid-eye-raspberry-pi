#include "amg88xx-i2c.h"


int main(int argc, char* argv[]) {
    Adafruit_AMG88xx grideye;
    grideye.init();

    float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
    grideye.readPixels(pixels,(uint8_t)sizeof(pixels));

    for (int r=0;r<8;r++) {
        for (int c=0;c<8;c++) {
            float val = pixels[r*8+c];
            printf("%.1f ",val);
        }
       printf("\n");
    }
    return 0;
}

