// Circle Class
class Circle {
   // Place private variables here.
   int radius;
   
  // Everything after "public" can be accesed outside of the class (i.e in "setup" or "loop")
  public:
    // Put a constructor here: What information is needed?
    Circle(int);
    
    // Create a function that returns area (use 3.14 for pi)
    double getArea(){
      return 3.14 * radius * radius;
    }
    // Create a function that allows the user to update the values
    void setRadius(int);
};

// Definitions of functions declared above
Circle::Circle(int r){
  radius = r;
}

void Circle::setRadius(int r){
  radius = r;
}

void setup() {
  // Instantiates the Circle class
  Circle c(10);
  
  // Can only print after serial.begin()
  Serial.begin(9600);
  
  // Calls the area() function within the class
  double area = c.getArea();
  
  // Prints area
  Serial.println(area);
  
  // Updates values
  c.setRadius(20);
  
  // Prints new areas
  area = c.getArea();
  Serial.println(area);
}

void loop() {
  // put your main code here, to run repeatedly:
}
