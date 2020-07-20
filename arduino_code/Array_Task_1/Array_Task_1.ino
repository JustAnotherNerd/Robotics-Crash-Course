void setup() {
  Serial.begin(9600);
  int arr[10];
  int i = 0;
  while(i < 10){
    arr[i] = 3 * (i + 1);
    i++;
  }

  for(int j = 1; j < 11; j++){
    arr[j - 1] = 3 * j;
  }

  for(int k = 0; k < 10; k++){
    if(arr[k] % 9 == 0){
      Serial.println(arr[k]);
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
