static char** bitBuffer;

void LCD5110_Init(){
  bitBuffer = malloc(sizeof(char*) * 84);
  for(int i = 0; i < 84; i++){
    bitBuffer[i] = malloc(sizeof(char) * 48);
  }
}

void LCD5110_writeCommand(uint8_t command){
  //Writes out the command in u8 command

}

void LCD5110_writeByte(uint8_t byte){
  //Writes this byte

}
