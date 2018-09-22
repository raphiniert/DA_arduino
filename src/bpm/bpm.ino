/*
  AnalogReadSerial

  Reads an analog input on pin 0, prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/AnalogReadSerial
*/

const int valArraySIze = 21;
int sensorValue = 0;
int oldValue = -1;
int bpm = 0;
int oldBpm = -1;
int bpmMin = 60;
int bpmMax = 200;
int minSensorVal = 0;
int maxSensorVal = 674;
int valArray[valArraySIze] = {};

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  for(int i = 0; i < valArraySIze; i++){
    valArray[i] = map(analogRead(A0), minSensorVal, maxSensorVal, bpmMax, bpmMin);
    delay(1);
  }

  quickSort(valArray, 0, valArraySIze - 1);

  bpm = valArray[(valArraySIze - 1)/2]; // get median value
  
  while(Serial.availableForWrite() <= 0){
      ; // wait
  }
  if(bpm != oldBpm){
    Serial.print(bpm);
    Serial.println(";");
    oldBpm = bpm;
  } else {
    //Serial.println("");
    ;
    delay(10);
  }
  //delay(10);
}

void quickSort(int arr[], int left, int right) {
     int i = left, j = right;
     int tmp;
     int pivot = arr[(left + right) / 2];

     /* partition */
     while (i <= j) {
           while (arr[i] < pivot)
                 i++;
           while (arr[j] > pivot)
                 j--;
           if (i <= j) {
                 tmp = arr[i];
                 arr[i] = arr[j];
                 arr[j] = tmp;
                 i++;
                 j--;
           }
     };

     /* recursion */
     if (left < j)
           quickSort(arr, left, j);
     if (i < right)
           quickSort(arr, i, right);
}
