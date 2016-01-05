#define channumber 5 //No. of PPM channels
int value[channumber];

void setup()
{
Serial.begin(115200); //Initialise serial interface
pinMode(3, INPUT); //Pin 3 as input
}

void loop()
{
while(pulseIn(3, LOW) < 5000){} //Wait for the beginning of the frame

for(int i=0; i<=channumber-1; i++)//Record channel data
{
value[i]=pulseIn(3, LOW);
}

for(int i=0; i<=channumber-1; i++)//Loop to print and clear all the channel readings
{
Serial.print(value[i]); //Print the values
Serial.print(" ");
value[i]=0; //Clear the value after it's printed
}

Serial.println(""); //Start a new line
}
