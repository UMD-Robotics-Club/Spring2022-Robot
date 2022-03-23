// This is a repository of useful code snippets



// High performance function to read serial data
bool recInProg = false;
bool new_data = false;
bool transmission_finished = true;
const int num_chars = 30;
char data[num_chars] = "";

void readSerial(bool* recvInProgress, bool* new_data, bool* transaction_finnished, char* text_data, int num_chars){
  *recvInProgress = false;
  static int ndx = 0;
  char startMarker = '!';
  char endMarker = ';';
  char c;
 
  while (Serial.available() > 0 && new_data == false) {
    c = Serial.read();
    if(c == '\n'){transmission_finished = true;}
    if (recvInProgress == true) {
      if (c != endMarker) {
        *text_data[ndx] = c;
        ndx++;
        if (ndx >= num_chars) {
          ndx = num_chars - 1;
        }
      }
      else {
        *text_data[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        new_data = true;
      }
    }
    else if (c == startMarker) {
        recvInProgress = true;
    }
  }
}



void loop(){
    readSerial(&recInProg);

    // do other stuff here

}