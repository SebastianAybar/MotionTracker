#include <stdio.h>  // Für Dateioperationen (fopen, fread, fclose, printf)
#include <stdlib.h> // Für Speicherverwaltung (malloc, free) und Umwandlungen (atoi)
#include <string.h> // Für String-Operationen (strtok, memset, strcpy)
#include <stdint.h> // Für feste Datentypen (uint8_t, int16_t)
#include <math.h>
#include "skript.h"

// Determine_Activities
uint16_t accelerationValueCounter = 0;
int32_t sumAccX = 0;
int32_t sumAccY = 0;
int32_t sumAccZ = 0;
int16_t accX[ACCELEROMETER_VALUES_PER_SIGNAL] = {0};
int16_t accY[ACCELEROMETER_VALUES_PER_SIGNAL] = {0};
int16_t accZ[ACCELEROMETER_VALUES_PER_SIGNAL] = {0};
uint16_t receivedSignalsCounter = 0;

uint8_t yAxisOrientation = yUndefined;
uint8_t xyAxisOrientation = xyUndefined;
int counterForFirstBlock = 0;
// Jumps
uint8_t yRange4jumpsReached = False;
uint8_t counter4valuesInRangeJumps = 0;
uint16_t jumps = 0;

// Stairs
uint16_t stairs = 0;
uint8_t stairStepDetected = False;
uint16_t stairStepCounter = 0;
#define STAIR_STEP_MIN_DURATION 30  // Mindestanzahl an Messungen zwischen zwei Stufen
#define STAIR_STEP_Y_THRESHOLD 1500 // Y-Schwellenwert für Stufenerkennung
#define STAIR_STEP_X_THRESHOLD -300 // X-Schwellenwert für Stufenerkennung

// Fall detection
uint8_t fallWatchCounter = 0;
uint16_t absMaxX = 0;
uint16_t absMaxY = 0;
uint16_t absMaxZ = 0;
uint16_t prevAbsMaxX = 0;
uint16_t prevAbsMaxY = 0;
uint16_t prevAbsMaxZ = 0;
uint8_t fallLikeAccelerations = False;
uint8_t fallDetected = False;
uint8_t fallConfirmed = False;
uint8_t uprightAfterFall = False;
uint8_t remainedUprightAfterFall = False;
uint16_t fallDetectCounter = 0;
int32_t sumAccY4Detect = 0;
uint16_t fallConfCounter = 0;
int32_t sumAccY4Fall = 0;

int main()
{
    // char file_name[] = "C:/Users/sebas/OneDrive/Dokumente/GitHub/InformatikProjekt/Daniel Helmer/Tracker/Dateien/Rohdaten/rawData_stairs_belt.txt";
    char file_name[] = "C:/Users/sebas/OneDrive/Dokumente/GitHub/InformatikProjekt/Daniel Helmer/Tracker/Dateien/Rohdaten/rawData_skipping_belt.txt";
    // Datei im Lesemodus öffnen
    FILE *file = fopen(file_name, "r");
    if (file == NULL)
    {
        perror("Fehler beim Öffnen der Datei");
        return 1;
    }

    // Den gesamten Inhalt der Datei lesen
    char buffer[MAX_FILE_SIZE];                                   // Puffer zum Speichern des Dateiinhalts
    size_t bytesRead = fread(buffer, 1, MAX_FILE_SIZE - 1, file); // Datei lesen
    buffer[bytesRead] = '\0';                                     // Null-Terminierung für den Puffer, um ihn als String zu behandeln

    fclose(file); // Datei schließen

    // Alle Leerzeichen entfernen
    char result[MAX_FILE_SIZE]; // Ein zweiter Puffer zum Speichern des bereinigten Inhalts
    int j = 0;                  // Index für den neuen Puffer
    int countRawData = 0;
    for (int i = 0; i < bytesRead; i++)
    {
        if (buffer[i] == '\n')
        {
            result[j++] = ','; // Ersetze Zeilenumbruch durch Komma
        }
        else if (buffer[i] != ' ')
        {
            result[j++] = buffer[i]; // Normales Zeichen übernehmen
        }
        if (buffer[i] == ',')
        {
            countRawData++; // Zähle die Werte basierend auf den Kommas
        }
    }
    // countRawData++;    // Die Anzahl der Rohdaten wird durch die Kommas in der Datei ermittelt, daher für die letzte Zahl + 1
    printf("gezaehlte Werte in Textdatei: %d \n", countRawData);
    result[j] = '\0'; // Null-Terminierung des bereinigten Puffers

    // printf("Erste 1000 Zeichen des bereinigten Inhalts:\n%.1000s\n", result);

    // Integer Array für die umgewandelten Werte
    int values[countRawData];
    int count = 0;
    // String in einzelne Werte aufteilen und in Integer umwandeln
    char *token = strtok(result, ","); // Trennen an den Kommas
    while (token != NULL && count < countRawData)
    {
        values[count++] = atoi(token); // In Integer umwandeln und speichern
        token = strtok(NULL, ",");     // Nächstes Token holen
    }

    // Values ausgeben
    // for (int i = 0; i < 100; i++) {
    //     printf("Value[%d]: %d\n", i, values[i]);
    // }

    int num_blocks = (int)(countRawData / 16);
    printf("num_blocks = %d \n", num_blocks);

    // Array für die vorzeichenbehafteten Beschleunigungswerte
    int16_t value_acc_x_signed[num_blocks - 1];
    int16_t value_acc_y_signed[num_blocks - 1];
    int16_t value_acc_z_signed[num_blocks - 1];

    // Schleife zur Verarbeitung der Daten in 16er-Blöcken
    for (int i = 0; i < num_blocks; i++)
    {
        // Accelerometer Values
        uint8_t acc_x_bytes_1 = (uint8_t)values[16 * i + 4];
        uint8_t acc_x_bytes_2 = (uint8_t)values[16 * i + 5];
        // Little-Endian-Kombination der beiden Bytes zu einem 16-Bit-Wert
        int16_t acc_x_signed_int = (int16_t)((acc_x_bytes_1 << 8) | acc_x_bytes_2);
        // Speichern im Array
        value_acc_x_signed[i] = acc_x_signed_int;

        uint8_t acc_y_bytes_1 = (uint8_t)values[16 * i + 6];
        uint8_t acc_y_bytes_2 = (uint8_t)values[16 * i + 7];
        int16_t acc_y_signed_int = (int16_t)((acc_y_bytes_1 << 8) | acc_y_bytes_2);
        value_acc_y_signed[i] = acc_y_signed_int;

        uint8_t acc_z_bytes_1 = (uint8_t)values[16 * i + 8];
        uint8_t acc_z_bytes_2 = (uint8_t)values[16 * i + 9];
        int16_t acc_z_signed_int = (int16_t)((acc_z_bytes_1 << 8) | acc_z_bytes_2);
        value_acc_z_signed[i] = acc_z_signed_int;
    }

    int size_of_acc_x = sizeof(value_acc_x_signed) / sizeof(value_acc_x_signed[0]);
    ;
    printf("size of each acc array = %d \n", size_of_acc_x);
    int size_of_acc_y = sizeof(value_acc_y_signed) / sizeof(value_acc_y_signed[0]);
    printf("size of each acc array = %d \n", size_of_acc_y);
    int size_of_acc_z = sizeof(value_acc_z_signed) / sizeof(value_acc_z_signed[0]);
    ;
    printf("size of each acc array = %d \n", size_of_acc_z);

    for (int i = 0; i < num_blocks; i++)
    {
        printf("Triple %d: ", i);
        printf("%d ", value_acc_x_signed[i]);
        printf("%d ", value_acc_y_signed[i]);
        printf("%d \n", value_acc_z_signed[i]);
    }

    for (int i = 0; i <= size_of_acc_y; i++)
    {
        Determine_Activities(value_acc_x_signed[i], value_acc_y_signed[i], value_acc_z_signed[i]);
        // printf("i: %d\n", i);
    }

    return 0;
}

void Determine_Activities(int16_t accelerationValueX, int16_t accelerationValueY, int16_t accelerationValueZ)
{
    // printf("accX: %d, accY: %d, accZ: %d\n", accelerationValueX, accelerationValueY, accelerationValueZ);

    uint8_t yAxisOrientationNew = yUndefined;
    uint8_t xyAxisOrientationNew = xyUndefined;
    uint8_t postureNew = postureUndefined;
    int16_t meanX = 0;
    int16_t meanY = 0;
    int16_t meanZ = 0;

    accX[accelerationValueCounter] = accelerationValueX;
    accY[accelerationValueCounter] = accelerationValueY;
    accZ[accelerationValueCounter] = accelerationValueZ;

    sumAccX += accelerationValueX;
    sumAccY += accelerationValueY;
    sumAccZ += accelerationValueZ;

    accelerationValueCounter++;
    // printf("accelerationValueCounter: %d \n", accelerationValueCounter);

    // if(counterForFirstBlock < 256) {
    //     yAxisOrientation = Get_Y_Axis_Orientation(accelerationValueY);
    // }
    // counterForFirstBlock++;

    // Methoden zur Bewegungserkennung
    Count_Jumps(accelerationValueY, yAxisOrientation);
    Detect_Fall(accelerationValueX, accelerationValueY, accelerationValueZ);
    Count_Stairs(accelerationValueX, accelerationValueY, accelerationValueZ, yAxisOrientation);

    // Wenn wir genug Messwerte haben
    if (accelerationValueCounter == ACCELEROMETER_VALUES_PER_SIGNAL)
    {
        // printf("test\n");
        // exit();
        accelerationValueCounter = 0;
        receivedSignalsCounter++;
        // Calculate mean for each axis (meanX = sumAccX / ACCELEROMETER_VALUES_PER_SIGNAL)
        meanX = sumAccX >> 8;
        meanY = sumAccY >> 8;
        meanZ = sumAccZ >> 8;

        // printf("meanY: %d \n", meanY);
        // printf("sumAccY: %d \n", sumAccY);

        sumAccX = 0;
        sumAccY = 0;
        sumAccZ = 0;

        yAxisOrientationNew = Get_Y_Axis_Orientation(meanY);
        // printf("y-AxisorientationNew = %d\r\n", yAxisOrientationNew);
        // xyAxisOrientationNew = Get_XY_Axis_Orientation(meanX, meanY);

        if (yAxisOrientation != yAxisOrientationNew && yAxisOrientationNew != yUndefined)
        {
            yAxisOrientation = yAxisOrientationNew;
            // printf("y-Axisorientation = %d\r\n", yAxisOrientation);
        }

        if (xyAxisOrientation != xyAxisOrientationNew && xyAxisOrientationNew != xyUndefined)
        {
            xyAxisOrientation = xyAxisOrientationNew;
            printf("xy-Axisorientation = %d\r\n", xyAxisOrientation);
        }

        // postureNew = Get_Posture(meanX, meanY, meanZ, xyAxisOrientation);

        // saveData();
    }
}

uint16_t Count_Stairs(int16_t meanX, int16_t meanY, int16_t meanZ, uint8_t yAxisOrientation)
{
    // Nur zählen wenn die Y-Achse nach oben zeigt (Person steht aufrecht)
    if (yAxisOrientation == yUp)
    {
        // printf("yUp\n");
        // Stufenmuster erkannt (Y > 1500 und X < -300)
        if (meanY < 3000 && meanY > 1500 && meanX < -300)
        {
            // printf("stairStepCounter: %d \n", stairStepCounter);
            if (stairStepDetected == False && stairStepCounter > STAIR_STEP_MIN_DURATION)
            {
                stairs++;
                printf("Stairs count: %d\r\n", stairs);
                stairStepDetected = True;
                stairStepCounter = 0;
                // printf("-------------------------------------------");
            }
        }
        else
        {
            stairStepDetected = False;
        }
        stairStepCounter++;
        // printf("%d\n", stairStepCounter);
    }
    else
    {
        // printf("False\n");
        // Zurücksetzen wenn nicht aufrecht
        stairStepDetected = False;
        stairStepCounter = 0;
    }
    return stairs;

    if (yAxisOrientation == yDown)
    {
        // printf("yUp\n");
        // Stufenmuster erkannt (Y > 1500 und X < -300)
        if (meanY > -3000 && meanY < -1500 && meanX > 300)
        {
            // printf("stairStepCounter: %d \n", stairStepCounter);
            if (stairStepDetected == False && stairStepCounter > STAIR_STEP_MIN_DURATION)
            {
                stairs++;
                printf("Stairs count: %d\r\n", stairs);
                stairStepDetected = True;
                stairStepCounter = 0;
                // printf("-------------------------------------------");
            }
        }
        else
        {
            stairStepDetected = False;
        }
        stairStepCounter++;
        // printf("%d\n", stairStepCounter);
    }
    else
    {
        // printf("False\n");
        // Zurücksetzen wenn nicht aufrecht
        stairStepDetected = False;
        stairStepCounter = 0;
    }
    return stairs;
}


uint8_t Get_Posture(int16_t meanX, int16_t meanY, int16_t meanZ, uint8_t xyAxisOrientation)
{
}

uint8_t Get_Y_Axis_Orientation(int16_t meanY)
{
    if (meanY > 800)
    {
        return yUp;
    }
    else if (meanY < -800)
    {
        return yDown;
    }
    else
    {
        return yUndefined;
    }
}

uint8_t Get_XY_Axis_Orientation(int16_t meanY, int16_t meanX)
{
    if (meanY > 800 && meanX < 0)
    {
        return yUpXForward;
    }
    if (meanY > 800 && meanX > 0)
    {
        return yUpXBackward;
    }
    if (meanY < -800 && meanX < 0)
    {
        return yDownXForward;
    }
    if (meanY < -800 && meanX > 0)
    {
        return yDownXBackward;
    }
    return xyUndefined;
}

void Count_Jumps(int16_t accelerationValueY, uint8_t yAxisOrientation)
{
    if (yAxisOrientation == yUp)
    {
        if (accelerationValueY > 4000)
        {
            yRange4jumpsReached = True;
        }
        if (yRange4jumpsReached == True)
        {
            counter4valuesInRangeJumps++;
        }
        if (yRange4jumpsReached == True && accelerationValueY < 100)
        {
            jumps++;
            printf("jumps = %d\r\n", jumps);
            yRange4jumpsReached = False;
            counter4valuesInRangeJumps = 0;
        }
        if (counter4valuesInRangeJumps > 40)
        {
            jumps++;
            printf("jumps = %d\r\n", jumps);
            yRange4jumpsReached = False;
            counter4valuesInRangeJumps = 0;
        }
    }
    if (yAxisOrientation == yDown)
    {
        if (accelerationValueY < -4000)
        {
            yRange4jumpsReached = True;
        }
        if (yRange4jumpsReached == True)
        {
            counter4valuesInRangeJumps++;
        }
        if (yRange4jumpsReached == True && accelerationValueY > -100)
        {
            jumps++;
            printf("jumps = %d\r\n", jumps);
            yRange4jumpsReached = False;
            counter4valuesInRangeJumps = 0;
        }
        if (counter4valuesInRangeJumps > 40)
        {
            jumps++;
            printf("jumps = %d\r\n", jumps);
            yRange4jumpsReached = False;
            counter4valuesInRangeJumps = 0;
        }
    }
}

void Detect_Fall(int16_t accelerationValueX, int16_t accelerationValueY, int16_t accelerationValueZ)
{
    int16_t meanAccY = 0;
    int16_t meanAccYDetect;

    if (abs(accelerationValueX) > absMaxX)
    {
        absMaxX = abs(accelerationValueX);
    }
    if (abs(accelerationValueY) > absMaxY)
    {
        absMaxY = abs(accelerationValueY);
    }
    if (abs(accelerationValueZ) > absMaxZ)
    {
        absMaxZ = abs(accelerationValueZ);
    }

    if (fallWatchCounter < 25)
    {
        fallWatchCounter++;
    }
    else
    {
        fallWatchCounter = 0;
        if ((absMaxX > 1000 || prevAbsMaxX > 1000) && (absMaxY > 1000 || prevAbsMaxY > 1000) && (absMaxZ > 1000 || prevAbsMaxZ > 1000))
        {
            fallLikeAccelerations = True;
        }
        prevAbsMaxX = absMaxX;
        prevAbsMaxY = absMaxY;
        prevAbsMaxZ = absMaxZ;
        absMaxX = 0;
        absMaxY = 0;
        absMaxZ = 0;
    }

    if (fallLikeAccelerations == True)
    {
        fallDetectCounter++;
        // Don´t use the first 52 values for calculation of meanAccYDetect. 564 - 52 = (1<<9)
        if (fallDetectCounter > 52 && fallDetectCounter < 564)
        {
            sumAccY4Detect += accelerationValueY;
            fallDetectCounter++;
            if (fallDetectCounter == 564)
            {
                meanAccYDetect = sumAccY4Detect >> 9;
                sumAccY4Detect = 0;
                fallDetectCounter = 0;
                fallLikeAccelerations = False;
                if (meanAccYDetect < 300 && meanAccYDetect > -300)
                {
                    fallDetected = True;
                }
            }
        }
    }

    if (fallDetected == True && fallConfirmed == False)
    {
        fallConfCounter++;
        sumAccY4Fall += accelerationValueY;
        if (fallConfCounter == 2048)
        {
            meanAccY = sumAccY4Fall >> 11;
            sumAccY4Fall = 0;
            fallConfCounter = 0;
            if (meanAccY < 300 && meanAccY > -300)
            {
                fallConfirmed = True;
                printf("Sturz bestätigt."); // Replace by a method to send a message
                fallDetected = False;
                remainedUprightAfterFall = False;
            }
        }
    }
}
