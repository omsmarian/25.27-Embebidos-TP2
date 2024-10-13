void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  //EASVVV
  int count = 0;
  for(int i=0;i<7;i++)
  {
    for(int j=0;j<3;j++)
    {
      if(j==0)
      {
        Serial.println(String(i)+"O+"+String( random(0, 255)));
      }
      else if(j==1)
      {
        Serial.println(String(i)+"H+"+String( random(0, 255)));
        if(count > 30)
        {
          count = 0;
        }
      }
      else
      {
        Serial.println(String(i)+"R+"+String( random(0, 255)));
      }
      
      delay(10);
    }
  }
}
