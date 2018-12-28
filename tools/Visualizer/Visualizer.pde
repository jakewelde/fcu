import processing.serial.*;
Serial myPort;

//float yaw = 0.0;
//float pitch = 0.0;
//float roll = 0.0;

void setup()
{
  size(600, 500, P3D);

  myPort = new Serial(this, "/dev/cu.usbmodem14201", 115200);

  textSize(16); // set text size
  textMode(SHAPE); // set text mode to shape
}

float R[][] =  { {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };   
float Om[ ] = {0,0,0};

void draw()
{
  serialEvent();  // read and parse incoming serial message
  background(255); // set background to white
  lights();

  translate(width/2, height/2); // set position to centre
  rotateX(-PI/2); // for camera perspective

  pushMatrix();
  
  // Processing uses a left-handed coordinate system, who are these people??? 
  applyMatrix( -R[0][0], -R[0][1], -R[0][2], 0,
               -R[1][0], -R[1][1], -R[1][2], 0,
               -R[2][0], -R[2][1], -R[2][2], 0,
               0, 0, 0, 1);

  drawArduino();

  popMatrix();

  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      print(R[r][c]);
      print("\t");
    }
    println();
  }
  print("Om: ");
  for (int c = 0; c < 3; c++) {
    print(Om[c]);
    print("\t");
  }
  println();
}

void serialEvent()
{
  int newLine = 13; // new line character in ASCII
  String message;
  do {
    message = myPort.readStringUntil(newLine); // read from port until new line
    if (message != null) {
      String[] list = split(trim(message), "\t");
      if (list.length >= 13 && list[0].equals("Orientation:")) {
        R[0][0] = float(list[1]); // convert to float yaw
        R[0][1] = float(list[2]); // convert to float yaw
        R[0][2] = float(list[3]); // convert to float yaw
        R[1][0] = float(list[4]); // convert to float yaw
        R[1][1] = float(list[5]); // convert to float yaw
        R[1][2] = float(list[6]); // convert to float yaw
        R[2][0] = float(list[7]); // convert to float yaw
        R[2][1] = float(list[8]); // convert to float yaw
        R[2][2] = float(list[9]); // convert to float yaw
        Om[0]   = float(list[10]); // convert to float yaw
        Om[1]   = float(list[11]); // convert to float yaw
        Om[2]   = float(list[12]); // convert to float yaw
      }
    }
  } while (message != null);
}

void drawArduino()
{
  
  fill(200, 200, 200); // set outline colour to darker teal
  stroke(10); // set fill colour to lighter teal
  box(100, 100, 5); // draw Arduino board base shape

  stroke(255,0,0);
  line(0,0,0,100,0,0);
  stroke(0,255,0);
  line(0,0,0,0,100,0);
  stroke(0,0,255);
  line(0,0,0,0,0,100);
}