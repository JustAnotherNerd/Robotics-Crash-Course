// Array Task 1

void setup() {
  Serial.begin(9600);

  // Creat an array of type int and size 10:
  int arr[10];
  
  // Using a while loop, fill the array with multiples of 3 between 3 and 30 (inclusive):
  int i = 0;
  while(i < 10){
    arr[i] = 3 * (i + 1);
    i++;
  }

  // Rewrite the code above with a for loop:
  for(int j = 1; j < 11; j++){
    arr[j - 1] = 3 * j;
  }

  // Write a loop that will go through the array 
  for(int k = 0; k < 10; k++){
    if(arr[k] % 9 == 0){
      Serial.println(arr[k]);
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
